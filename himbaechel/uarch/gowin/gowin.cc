#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <map>
#include <regex>
#include <vector>

#include "himbaechel_api.h"
#include "himbaechel_helpers.h"
#include "log.h"
#include "nextpnr.h"

#define GEN_INIT_CONSTIDS
#define HIMBAECHEL_CONSTIDS "uarch/gowin/constids.inc"
#include "himbaechel_constids.h"

#include "array2d.h"
#include "cst.h"
#include "globals.h"
#include "gowin.h"
#include "gowin_utils.h"
#include "pack.h"

#include "placer_heap.h"

// Codex round 12 Step 4: SEL[i] rejection counter (instrumentation)
namespace { std::atomic<uint64_t> g_sel_i_reject_count{0}; }

NEXTPNR_NAMESPACE_BEGIN

namespace {
struct GowinImpl : HimbaechelAPI
{

    ~GowinImpl() {};
    po::options_description getUArchOptions() override;
    void init_database(Arch *arch) override;
    void init(Context *ctx) override;

    void pack() override;
    void prePlace() override;
    void postPlace() override;
    void preRoute() override;
    void postRoute() override;

    bool isBelLocationValid(BelId bel, bool explain_invalid) const override;
    void notifyBelChange(BelId bel, CellInfo *cell) override;

    delay_t estimateDelay(WireId src, WireId dst) const override;

    // Bel bucket functions
    IdString getBelBucketForCellType(IdString cell_type) const override;

    bool isValidBelForCellType(IdString cell_type, BelId bel) const override;

    // wires
    bool checkPipAvail(PipId pip) const override;
    bool checkPipAvailForNet(PipId pip, const NetInfo *net) const override;

    // Cluster
    bool isClusterStrict(const CellInfo *cell) const override { return true; }
    bool getClusterPlacement(ClusterId cluster, BelId root_bel,
                             std::vector<std::pair<CellInfo *, BelId>> &placement) const override;

    void configurePlacerHeap(PlacerHeapCfg &cfg) override;

    void drawBel(std::vector<GraphicElement> &g, GraphicElement::style_t style, IdString bel_type, Loc loc) override;

  private:
    HimbaechelHelpers h;
    GowinUtils gwu;

    IdString chip;
    IdString partno;

    std::set<BelId> inactive_bels;

    // Validity checking
    struct GowinCellInfo
    {
        // slice info
        const NetInfo *lut_f = nullptr;
        const NetInfo *ff_d = nullptr, *ff_ce = nullptr, *ff_clk = nullptr, *ff_lsr = nullptr;
        const NetInfo *alu_sum = nullptr;
        // dsp info
        const NetInfo *dsp_asign = nullptr, *dsp_bsign = nullptr, *dsp_asel = nullptr, *dsp_bsel = nullptr,
                      *dsp_ce = nullptr, *dsp_clk = nullptr, *dsp_reset = nullptr;
        const NetInfo *dsp_5a_clk0 = nullptr, *dsp_5a_clk1 = nullptr, *dsp_5a_ce0 = nullptr, *dsp_5a_ce1 = nullptr,
                      *dsp_5a_reset0 = nullptr, *dsp_5a_reset1 = nullptr;
        bool dsp_soa_reg;
    };
    std::vector<GowinCellInfo> fast_cell_info;
    void assign_cell_info();

    // If there is an unused LUT adjacent to FF, use it
    void create_passthrough_luts(void);

    // Remember HCLK sections that have been reserved to route HCLK signals
    std::set<BelId> routing_reserved_hclk_sections;

    // dsp control nets
    // Each DSP and each macro has a small set of control wires that are
    // allocated to internal primitives as needed. It is assumed that most
    // primitives use the same signals for CE, CLK and especially RESET, so
    // these wires are few and need to be controlled.
    struct dsp_info
    {
        dict<IdString, int> ce;    // CE nets counter
        dict<IdString, int> clk;   // CLK nets counter
        dict<IdString, int> reset; // RESET nets counter
        int mode9bit;              // 9 bit elements counter
        int mode18bit;             // 18 bit elements counter
    };
    dict<BelId, dsp_info> dsp_info;
    dict<BelId, CellInfo *> dsp_bel2cell; // Remember the connection with cells
                                          // since this information is already lost during unbinding
    void adjust_dsp_pin_mapping(void);

    // Place explicityl constrained or implicitly constrained (by IOLOGIC) CLKDIV and CLKDIV2 cells
    // to avoid routing conflicts and maximize utilization
    void place_constrained_hclk_cells();
    void place_5a_hclks(void);
    void constrain_r56_fabric_dq_capture_ffs(void);
    void log_r57_preplace_debug_ffs(void);

    // bel placement validation
    bool slice_valid(int x, int y, int z) const;
    bool dsp_valid(Loc l, IdString bel_type, bool explain_invalid) const;
    bool hclk_valid(BelId bel, IdString bel_type) const;

    void prepare_fast_logic_cell();

    array2d<std::vector<CellInfo *>> fast_logic_cell;

    delay_t delay_m, delay_c;
};

struct GowinArch : HimbaechelArch
{
    GowinArch() : HimbaechelArch("gowin") {};

    bool match_device(const std::string &device) override { return device.size() > 2 && device.substr(0, 2) == "GW"; }

    std::unique_ptr<HimbaechelAPI> create(const std::string &device) override { return std::make_unique<GowinImpl>(); }
} gowinArch;

po::options_description GowinImpl::getUArchOptions()
{
    po::options_description specific("Gowin specific options");
    specific.add_options()("family", po::value<std::string>(), "GOWIN chip family");
    specific.add_options()("cst", po::value<std::string>(), "name of constraints file");
    specific.add_options()("ireg_in_iob", "place input registers in IOB");
    specific.add_options()("oreg_in_iob", "place output registers in IOB");
    specific.add_options()("ioreg_in_iob", "place I/O registers in IOB");
    specific.add_options()("disable_gp_clock_routing", "disable clock network routing from GP pins");
    specific.add_options()("sspi_as_gpio", "use SSPI pins as GPIO");
    specific.add_options()("i2c_as_gpio", "use I2C pins as GPIO");
    return specific;
}

void GowinImpl::init_database(Arch *arch)
{
    init_uarch_constids(arch);
    const ArchArgs &args = arch->args;
    std::string family;
    if (args.options.count("family")) {
        family = args.options["family"].as<std::string>();
    } else {
        bool GW2 = args.device.rfind("GW2A", 0) == 0;
        if (GW2) {
            log_error("For the GW2A series you need to specify --vopt family=GW2A-18 or --vopt family=GW2A-18C\n");
        } else {
            std::regex devicere = std::regex("GW5A(T|ST)?-LV(25|60|138)[A-Z]*.*");
            std::smatch match;
            if (std::regex_match(args.device, match, devicere)) {
                family = stringf("GW5A%s-%s%s", match[1].str().c_str(), match[2].str().c_str(),
                                 match[2].str() == "25" ? "A" : "C");
            } else {
                std::regex devicere = std::regex("GW1N([SZ]?)[A-Z]*-(LV|UV|UX)([0-9])(C?).*");
                std::smatch match;
                if (!std::regex_match(args.device, match, devicere)) {
                    log_error("Invalid device %s\n", args.device.c_str());
                }
                family = stringf("GW1N%s-%s", match[1].str().c_str(), match[3].str().c_str());
                if (family.rfind("GW1N-9", 0) == 0) {
                    log_error("For the GW1N-9 series you need to specify --vopt family=GW1N-9 or --vopt "
                              "family=GW1N-9C\n");
                }
            }
        }
    }

    arch->load_chipdb(stringf("gowin/chipdb-%s.bin", family.c_str()));

    // These fields go in the header of the output JSON file and can help
    // gowin_pack support different architectures
    arch->settings[arch->id("packer.arch")] = std::string("himbaechel/gowin");
    arch->settings[arch->id("packer.chipdb")] = family;

    chip = arch->id(family);
    std::string pn = args.device;
    partno = arch->id(pn);
    arch->settings[arch->id("packer.partno")] = pn;
}

void GowinImpl::init(Context *ctx)
{
    h.init(ctx);
    HimbaechelAPI::init(ctx);

    gwu.init(ctx);

    const ArchArgs &args = ctx->args;

    // package and speed class
    std::regex speedre = std::regex("(.*)(C[0-9]/I[0-9])$");
    std::smatch match;

    IdString spd;
    IdString package_idx;
    std::string pn = args.device;
    if (std::regex_match(pn, match, speedre)) {
        package_idx = ctx->id(match[1]);
        spd = ctx->id(match[2]);
        ctx->set_speed_grade(match[2]);
    } else {
        if (pn.length() > 2 && pn.compare(pn.length() - 3, 2, "ES")) {
            package_idx = ctx->id(pn.substr(0, pn.length() - 2));
            spd = ctx->id("ES");
            ctx->set_speed_grade("ES");
        }
    }

    // log_info("search for %s, packages:%ld\n", package_idx.c_str(ctx), ctx->chip_info->packages.ssize());
    for (int i = 0; i < ctx->chip_info->packages.ssize(); ++i) {
        // log_info("i:%d %s\n", i, IdString(ctx->chip_info->packages[i].name).c_str(ctx));
        if (IdString(ctx->chip_info->packages[i].name) == package_idx) {
            ctx->package_info = &ctx->chip_info->packages[i];
            break;
        }
    }
    if (ctx->package_info == nullptr) {
        log_error("No package for partnumber %s\n", partno.c_str(ctx));
    }

    // constraints
    if (args.options.count("cst")) {
        ctx->settings[ctx->id("cst.filename")] = args.options["cst"].as<std::string>();
    }

    // place registers in IO blocks
    if (args.options.count("ireg_in_iob")) {
        ctx->settings[id_IREG_IN_IOB] = Property(1);
    }
    if (args.options.count("oreg_in_iob")) {
        ctx->settings[id_OREG_IN_IOB] = Property(1);
    }
    if (args.options.count("ioreg_in_iob")) {
        ctx->settings[id_IOREG_IN_IOB] = Property(1);
    }

    // smart clock routing
    if (args.options.count("disable_gp_clock_routing")) {
        ctx->settings[id_NO_GP_CLOCK_ROUTING] = Property(1);
    }

    // configure delay estimates for A*
    if (args.device.rfind("GW2A", 0) == 0 || args.device.rfind("GW5A", 0) == 0) {
        delay_c = 300;
        delay_m = 60;
        ctx->ripup_penalty = 400;
    } else {
        delay_c = 600;
        delay_m = 120;
        ctx->ripup_penalty = 800;
    }
}

// We do not allow the use of global wires that bypass a special router.
bool GowinImpl::checkPipAvail(PipId pip) const
{
    return (ctx->getWireConstantValue(ctx->getPipSrcWire(pip)) != IdString()) ||
           (!(gwu.is_global_pip(pip) || gwu.is_segment_pip(pip)));
}

bool GowinImpl::checkPipAvailForNet(PipId pip, const NetInfo *net) const
{
    return (net->constant_value == IdString() && ctx->getWireConstantValue(ctx->getPipSrcWire(pip)) != IdString()) ||
           (!(gwu.is_global_pip(pip) || gwu.is_segment_pip(pip)));
}

void GowinImpl::pack()
{
    if (ctx->settings.count(ctx->id("cst.filename"))) {
        std::string filename = ctx->settings[ctx->id("cst.filename")].as_string();
        auto in = open_ifstream_and_log_error(filename, "CST file");

        if (!gowin_apply_constraints(ctx, in)) {
            log_error("failed to parse CST file '%s'\n", filename.c_str());
        }
    }
    gowin_pack(ctx);
}

// One DSP macro, in a rough approximation, consists of 5 large operating
// blocks (pre-adders, multipliers and alu), at almost every input (blocks
// usually have two of them) you can turn on registers, in addition, there are
// registers on a dedicated operand shift line between DSP and registers at
// the outputs. As we see, the number of registers is large, but the DSP has
// only four inputs for each of the CE, CLK and RESET signals, and here we tell
// gowin_pack which version of each signal is used by which block.
// We also indicate to the router which Bel's pin to use.
void GowinImpl::adjust_dsp_pin_mapping(void)
{
    if (gwu.has_5A_DSP()) {
        return;
    }
    for (auto b2c : dsp_bel2cell) {
        BelId bel = b2c.first;
        Loc loc = ctx->getBelLocation(bel);
        CellInfo *ci = b2c.second;
        const auto dsp_data = fast_cell_info.at(ci->flat_index);

        auto set_cell_bel_pin = [&](dict<IdString, int> nets, IdString pin, IdString net_name, const char *fmt,
                                    const char *fmt_double = nullptr) {
            int i = 0;
            for (auto net_cnt : nets) {
                if (net_cnt.first == net_name) {
                    break;
                }
                ++i;
            }
            ci->cell_bel_pins.at(pin).clear();
            if (fmt_double == nullptr) {
                ci->cell_bel_pins.at(pin).push_back(ctx->idf(fmt, i));
            } else {
                ci->cell_bel_pins.at(pin).push_back(ctx->idf(fmt_double, i, 0));
                ci->cell_bel_pins.at(pin).push_back(ctx->idf(fmt_double, i, 1));
            }
            ci->setAttr(pin, i);
        };

        if (dsp_data.dsp_reset != nullptr) {
            BelId dsp = ctx->getBelByLocation(Loc(loc.x, loc.y, BelZ::DSP_Z));
            set_cell_bel_pin(dsp_info.at(dsp).reset, id_RESET, dsp_data.dsp_reset->name, "RESET%d",
                             ci->type == id_MULT36X36 ? "RESET%d%d" : nullptr);
        }
        if (dsp_data.dsp_ce != nullptr) {
            BelId dsp = ctx->getBelByLocation(Loc(loc.x, loc.y, gwu.get_dsp_macro(loc.z)));
            set_cell_bel_pin(dsp_info.at(dsp).ce, id_CE, dsp_data.dsp_ce->name, "CE%d",
                             ci->type == id_MULT36X36 ? "CE%d%d" : nullptr);
        }
        if (dsp_data.dsp_clk != nullptr) {
            BelId dsp = ctx->getBelByLocation(Loc(loc.x, loc.y, gwu.get_dsp_macro(loc.z)));
            set_cell_bel_pin(dsp_info.at(dsp).clk, id_CLK, dsp_data.dsp_clk->name, "CLK%d",
                             ci->type == id_MULT36X36 ? "CLK%d%d" : nullptr);
        }
    }
}

// GW5A HCLK
// Each serializer/deserializer can use one of several HCLK wire from a block
// (hclk_idx), but only from the specific block assigned to that
// serializer/deserializer.
// Each HCLK line can be routed through a CLKDIV2 primitive, which halves the signal frequency.
//
// Thus, if the SERDES uses CLKDIV2, the latter must be placed in the specific
// HCLK block corresponding to the SERDES (since the SERDES is part of the I/O,
// and we do not allow unconstrained I/O, the placement of the SERDES is always
// known).
//
// Since users typically do not track which pins belong to which HCLK block,
// situations may arise where a single CLKDIV2 divider in the design serves as
// the signal source for SERDES located in different HCLK blocks, making it
// impossible to connect them. Alternatively, some SERDES in one block may
// receive the signal directly, while others receive it from the CLKDIV2.
//
// Here, we solve this by duplicating the CLKDIV2 cells and placing them
// exactly where they serve specific HCLK blocks.
//
// Information regarding which IO corresponds to a specific HCLK, as well as
// the placement of CLKDIV2 for that specific HCLK, is retrieved from the
// chip's database.
//
// CLKDIV can be used either on its own or in conjunction with CLKDIV2;
// however, in the latter case, there is only one CLKDIV2->CLKDIV connection,
// and it is non-switchable. This makes it necessary to clone CLKDIV2 when
// using it with multiple CLKDIVs.
//
void GowinImpl::place_5a_hclks(void)
{
    std::vector<std::unique_ptr<CellInfo>> new_cells;
    // Cloned CLKDIV2 cells
    dict<int, CellInfo *> clkdiv2_clones;
    // CLKDIV cells
    std::vector<CellInfo *> clkdiv_list;
    // Which users need to be reconnected, and to where.
    std::vector<std::pair<PortRef, CellInfo *>> users_to_reconect;
    // CLKDIV2 allocator
    dict<int, std::vector<Loc>> free_clkdiv2;

    // SERDES can use any wire from the HCLK block; here, we select a wire from
    // the unused ones and return the CLKDIV2 location that serves that wire.
    auto alloc_clkdiv2 = [&](int hclk_idx, CellInfo *ci) -> Loc {
        // first allocation for hclk_idx
        if (!free_clkdiv2.count(hclk_idx)) {
            std::vector<Loc> locs;
            gwu.get_clkdiv2_locs(hclk_idx, locs);
            free_clkdiv2[hclk_idx] = locs;
        }

        if (!free_clkdiv2.at(hclk_idx).size()) {
            log_error("Can't place %s CLKDIV2.\n", ctx->nameOf(ci));
        }
        Loc loc = free_clkdiv2.at(hclk_idx).back();
        free_clkdiv2.at(hclk_idx).pop_back();
        return loc;
    };

    // If CLKDIV is used together with CLKDIV2, we should place them together.
    auto make_clkdiv2_clkdiv_cluster = [&](CellInfo *ci, CellInfo *clkdiv) -> void {
        if (ctx->debug) {
            log_info("  Make cluster from %s and %s.\n", ctx->nameOf(ci), ctx->nameOf(clkdiv));
        }

        ci->cluster = ci->name;
        ci->constr_abs_z = false;
        ci->constr_children.push_back(clkdiv);

        clkdiv->cluster = ci->name;
        clkdiv->constr_abs_z = false;
        clkdiv->constr_x = 0;
        clkdiv->constr_y = 0;
        clkdiv->constr_z = BelZ::CLKDIV_0_Z - BelZ::CLKDIV2_0_Z;
    };

    for (auto &cell : ctx->cells) {
        auto ci = cell.second.get();

        // The CLKDIV2s described in the design
        if (is_clkdiv2(ci)) {
            NetInfo *hclk_net = ci->getPort(id_CLKOUT);
            if (!hclk_net || !ci->getPort(id_HCLKIN)) {
                continue;
            }
            if (ctx->debug) {
                log_info("  CLKDIV2 cell:%s, HCLKIN:%s, CLKOUT:%s\n", ctx->nameOf(ci),
                         ctx->nameOf(ci->getPort(id_HCLKIN)), ctx->nameOf(hclk_net));
            }

            clkdiv2_clones.clear();
            clkdiv_list.clear();
            users_to_reconect.clear();
            int cur_clkdiv2_hclk_idx = -1;

            // CLKDIV and SERDES only
            for (auto user : hclk_net->users) {
                if (is_clkdiv(user.cell)) {
                    clkdiv_list.push_back(user.cell);
                }
                if (!is_iologico(user.cell) && !is_iologici(user.cell)) {
                    continue;
                }
                // checking users' hclk index
                NPNR_ASSERT(user.cell->bel != BelId());
                Loc user_loc = ctx->getBelLocation(user.cell->bel);
                int hclk_idx = gwu.get_hclk_for_io(user_loc);
                if (hclk_idx == -1) {
                    log_error("%s can't use HCLK with %s.\n", ctx->nameOf(user.cell), ctx->nameOf(ci));
                }

                if (cur_clkdiv2_hclk_idx == -1) {
                    // Place CLKDIV2
                    cur_clkdiv2_hclk_idx = hclk_idx;
                    BelId bel = ctx->getBelByLocation(alloc_clkdiv2(hclk_idx, ci));
                    ctx->bindBel(bel, ci, PlaceStrength::STRENGTH_LOCKED);
                    if (ctx->debug) {
                        log_info("  @%s\n", ctx->nameOfBel(bel));
                    }
                    if (ctx->debug) {
                        log_info("    hclk:%d - %s %s\n", hclk_idx, ctx->nameOfBel(user.cell->bel),
                                 ctx->nameOf(user.cell));
                    }
                    continue;
                }

                // If the SERDES is in the current HCLK block, it remains there
                if (cur_clkdiv2_hclk_idx == hclk_idx) {
                    if (ctx->debug) {
                        log_info("    hclk:%d - %s %s\n", hclk_idx, ctx->nameOfBel(user.cell->bel),
                                 ctx->nameOf(user.cell));
                    }
                    continue;
                }

                // Since the SERDES is located in a different HCLK block, we
                // need to create a copy of CLKDIV2 and save the SERDES
                // for later connection to the copy.
                CellInfo *new_clkdiv2;
                if (clkdiv2_clones.count(hclk_idx)) {
                    // already have clone
                    new_clkdiv2 = clkdiv2_clones.at(hclk_idx);
                } else {
                    // create clone
                    new_cells.push_back(gwu.create_cell(gwu.create_aux_name(ci->name, hclk_idx), id_CLKDIV2));
                    new_clkdiv2 = new_cells.back().get();
                    new_clkdiv2->addInput(id_HCLKIN);
                    new_clkdiv2->addOutput(id_CLKOUT);
                    clkdiv2_clones[hclk_idx] = new_clkdiv2;
                    if (ctx->debug) {
                        log_info("    create clone for hclk:%d - %s\n", hclk_idx, ctx->nameOf(new_clkdiv2));
                    }
                }
                users_to_reconect.push_back(std::make_pair(user, new_clkdiv2));
            }
            // move the SERDES by connecting it to a copy of CLKDIV2
            for (auto us_ci : users_to_reconect) {
                PortRef user = us_ci.first;
                CellInfo *new_clkdiv2 = us_ci.second;
                if (ctx->debug) {
                    log_info("   reconnect %s.%s to %s\n", ctx->nameOf(user.cell), user.port.c_str(ctx),
                             ctx->nameOf(new_clkdiv2));
                }
                // input is same
                if (!new_clkdiv2->getPort(id_HCLKIN)) {
                    ci->copyPortTo(id_HCLKIN, new_clkdiv2, id_HCLKIN);
                }
                // move user
                user.cell->disconnectPort(user.port);
                new_clkdiv2->connectPorts(id_CLKOUT, user.cell, user.port);
            }

            // Place CLKDIV
            if (clkdiv_list.size()) {
                // First, the most common configuration: a single CLKDIV connected to former CLKDIV2
                CellInfo *clkdiv = clkdiv_list.back();
                clkdiv_list.pop_back();

                // Place CLKDIV only if CLKDIV2 is placed
                if (ci->bel != BelId()) {
                    Loc loc = ctx->getBelLocation(ci->bel);
                    loc.z = loc.z - BelZ::CLKDIV2_0_Z + BelZ::CLKDIV_0_Z;
                    BelId bel = ctx->getBelByLocation(loc);
                    ctx->bindBel(bel, clkdiv, PlaceStrength::STRENGTH_LOCKED);
                } else {
                    make_clkdiv2_clkdiv_cluster(ci, clkdiv);
                }

                // Connect the remaining CLKDIVs to the clones
                int name_sfx = 0;
                for (auto clkdiv : clkdiv_list) {
                    CellInfo *clone;
                    // If we have free clones
                    if (clkdiv2_clones.size()) {
                        clone = clkdiv2_clones.begin()->second;
                        clkdiv2_clones.erase(clkdiv2_clones.begin());
                    } else {
                        // create clone
                        ++name_sfx;
                        new_cells.push_back(
                                gwu.create_cell(gwu.create_aux_name(ci->name, name_sfx, "$for_clkdiv"), id_CLKDIV2));
                        clone = new_cells.back().get();
                        clone->addInput(id_HCLKIN);
                        clone->addOutput(id_CLKOUT);
                        // input is same
                        ci->copyPortTo(id_HCLKIN, clone, id_HCLKIN);
                    }
                    clkdiv->disconnectPort(id_HCLKIN);
                    clone->connectPorts(id_CLKOUT, clkdiv, id_HCLKIN);
                }
            }
        }
    }

    // Place new CLKDIV2
    for (auto &ncell : new_cells) {
        CellInfo *ci = ncell.get();
        NetInfo *hclk_net = ci->getPort(id_CLKOUT);
        if (ctx->debug) {
            log_info("  CLKDIV2 cell:%s, HCLKIN:%s, CLKOUT:%s\n", ctx->nameOf(ci), ctx->nameOf(ci->getPort(id_HCLKIN)),
                     ctx->nameOf(hclk_net));
        }

        // If we have only one user, and that user is CLKDIV, then there are no
        // strict restrictions on location; the only requirement is that they
        // be co-located — so we create a cluster.
        if (hclk_net->users.entries() == 1 && is_clkdiv((*hclk_net->users.begin()).cell)) {
            make_clkdiv2_clkdiv_cluster(ci, (*hclk_net->users.begin()).cell);
        } else {
            // Any user can be used to determine hclk_idx because we connected them
            // to copies of CLKDIV2 based precisely on the fact that hclk_idx is
            // the same
            PortRef &user = *hclk_net->users.begin();
            Loc user_loc = ctx->getBelLocation(user.cell->bel);
            int hclk_idx = gwu.get_hclk_for_io(user_loc);

            BelId bel = ctx->getBelByLocation(alloc_clkdiv2(hclk_idx, ci));
            ctx->bindBel(bel, ci, PlaceStrength::STRENGTH_LOCKED);
            if (ctx->debug) {
                log_info("  @%s\n", ctx->nameOfBel(bel));
                log_info("    hclk:%d - %s %s\n", hclk_idx, ctx->nameOfBel(user.cell->bel), ctx->nameOf(user.cell));
            }
            // Place CLKDIV if any
            for (auto user : hclk_net->users) {
                if (is_clkdiv(user.cell)) {
                    Loc loc = ctx->getBelLocation(ci->bel);
                    loc.z = loc.z - BelZ::CLKDIV2_0_Z + BelZ::CLKDIV_0_Z;
                    BelId bel = ctx->getBelByLocation(loc);
                    ctx->bindBel(bel, user.cell, PlaceStrength::STRENGTH_LOCKED);
                }
            }
        }
        ctx->cells[ncell->name] = std::move(ncell);
    }
}

/*
   Each HCLK section can serve one of three purposes:
       1. A simple routing path to IOLOGIC FCLK or PLL inputs
       2. CLKDIV2
       3. CLKDIV (only one section at any time)

   Our task is to distribute HCLK signal providers to sections in a way that maximizes utilization while
   enforcing user constraints on CLKDIV placement. We achieve this by solving two bipartite matchings:
   - The first determines the best HCLK to place a CLKDIV within the established graph. This is then refined
       to determine what section to assign the CLKDIV to based on what IOLOGIC it connects to
   - The second determines which HCLK sections to use as CLKDIV2 or to reserve for routing.
*/
void GowinImpl::place_constrained_hclk_cells()
{
    log_info("Running custom HCLK placer...\n");
    if (gwu.has_5A_HCLK()) {
        place_5a_hclks();
        return;
    }

    std::map<IdStringList, IdString> constrained_clkdivs;
    std::map<BelId, std::set<std::pair<IdString, int>>> bel_cell_map;
    std::vector<std::pair<IdString, int>> alias_cells;
    std::map<std::pair<IdString, int>, BelId> final_placement;

    const bool chip_has_clkdiv_hclk_connection = gwu.has_CLKDIV_HCLK();
    const bool chip_has_pll_hclk = gwu.has_PLL_HCLK();
    pool<BelId> free_pll_bels;
    if (chip_has_pll_hclk) {
        // gather free PLL bels
        for (auto bel : ctx->getBels()) {
            if (ctx->getBelType(bel).in(id_rPLL, id_PLLVR) && !ctx->getBoundBelCell(bel)) {
                free_pll_bels.insert(bel);
            }
        }
    }

    const auto is_hclk_user = [&](const CellInfo *ci) -> bool {
        if ((is_iologici(ci) || is_iologico(ci)) &&
            !ci->type.in(id_ODDR, id_ODDRC, id_IDDR, id_IDDRC, id_IOLOGICI_EMPTY, id_IOLOGICO_EMPTY)) {
            return true;
        }
        if (chip_has_pll_hclk && is_pll(ci)) {
            return true;
        }
        return false;
    };

    // returns the list of networks connected to the cell
    std::vector<const NetInfo *> net_list;
    auto get_nets = [&](const CellInfo *ci) -> void {
        net_list.clear();
        if (is_iologici(ci) || is_iologico(ci)) {
            net_list.push_back(ci->getPort(id_FCLK));
            return;
        }
        if (chip_has_pll_hclk && is_pll(ci)) {
            net_list.push_back(ci->getPort(id_CLKIN));
            net_list.push_back(ci->getPort(id_CLKFB));
            return;
        }
    };

    // cell, port
    std::set<IdString> seen_hclk_users;
    for (auto &cell : ctx->cells) {
        auto ci = cell.second.get();

        if (is_clkdiv(ci) && ci->attrs.count(id_BEL)) {
            BelId constrained_bel = ctx->getBelByName(IdStringList::parse(ctx, ci->attrs.at(id_BEL).as_string()));
            NPNR_ASSERT(constrained_bel != BelId() && ctx->getBelType(constrained_bel) == id_CLKDIV);
            auto hclk_id_loc = gwu.get_hclk_id(constrained_bel);
            constrained_clkdivs[hclk_id_loc] = ci->name;
        }

        if ((seen_hclk_users.find(ci->name) != seen_hclk_users.end()))
            continue;

        if (is_hclk_user(ci)) {
            get_nets(ci);
            for (auto &hclk_net : net_list) {
                if (!hclk_net) {
                    continue;
                }
                CellInfo *hclk_driver = hclk_net->driver.cell;
                if (!hclk_driver)
                    continue;
                if (!(chip_has_clkdiv_hclk_connection || hclk_driver->type == id_CLKDIV2)) {
                    continue;
                }

                int alias_count = 0;
                std::set<std::set<BelId>> seen_options;
                for (auto user : hclk_net->users) {
                    std::vector<BelId> bel_candidates;
                    std::set<BelId> these_options;

                    if (!is_hclk_user(user.cell)) {
                        continue;
                    }
                    seen_hclk_users.insert(user.cell->name);

                    if (ctx->debug) {
                        log_info("Custom HCLK Placer: Found HCLK user: %s\n", user.cell->name.c_str(ctx));
                    }

                    if (is_pll(user.cell) && user.cell->bel == BelId()) {
                        if (free_pll_bels.empty()) {
                            log_error("No BELs for %s\n", ctx->nameOf(user.cell));
                        }
                        ctx->bindBel(free_pll_bels.pop(), user.cell, PlaceStrength::STRENGTH_LOCKED);
                    }

                    gwu.find_connected_bels(user.cell, user.port, id_CLKDIV2, id_CLKOUT, 1000000, bel_candidates);
                    these_options.insert(bel_candidates.begin(), bel_candidates.end());

                    if (seen_options.find(these_options) != seen_options.end())
                        continue;
                    seen_options.insert(these_options);

                    // When an HCLK signal is routed to different (and disconnected) FCLKs, we treat each new
                    // HCLK-FCLK connection as a pseudo-HCLK cell since it must also be assigned an HCLK section
                    auto alias_index = std::pair<IdString, int>(hclk_driver->name, alias_count);
                    alias_cells.push_back(alias_index);
                    alias_count++;

                    for (auto option : these_options) {
                        bel_cell_map[option].insert(alias_index);
                    }
                }
            }
        }
    }

    // First matching. We use the upper CLKDIV2 as the ID for an HCLK
    std::map<IdStringList, std::set<IdString>> clkdiv_graph;
    for (auto bel_cell_candidates : bel_cell_map) {
        auto bel = bel_cell_candidates.first;
        auto hclk_id_loc = gwu.get_hclk_id(bel);
        if (constrained_clkdivs.find(hclk_id_loc) != constrained_clkdivs.end()) {
            continue;
        }
        for (auto candidate : bel_cell_candidates.second) {
            auto ci = ctx->cells.at(candidate.first).get();
            if ((ci->type != id_CLKDIV) || ci->attrs.count(id_BEL)) {
                continue;
            }
            clkdiv_graph[hclk_id_loc].insert(candidate.first);
        }
    }

    if (ctx->debug) {
        log_info("<-----CUSTOM HCLK PLACER: Constrained CLKDIVs----->\n");
        for (auto match_pair : constrained_clkdivs) {
            log_info("%s cell <-----> CLKDIV at HCLK %s\n", match_pair.second.c_str(ctx), match_pair.second.c_str(ctx));
        }
        log("\n");
    }

    auto matching = gwu.find_maximum_bipartite_matching<IdStringList, IdString>(
            clkdiv_graph); // these will serve as constraints
    constrained_clkdivs.insert(matching.begin(), matching.end());

    if (ctx->debug) {
        log_info("<-----CUSTOM HCLK PLACER: First Matching(CLKDIV) Results----->\n");
        for (auto match_pair : matching) {
            log_info("%s cell <-----> CLKDIV at HCLK %s\n", match_pair.second.c_str(ctx), match_pair.second.c_str(ctx));
        }
        log("\n");
    }

    // Refine matching to HCLK section, based on what connections actually exist
    std::map<IdString, std::pair<IdString, int>> true_clkdivs;
    std::set<BelId> used_bels;
    for (auto constr_pair : constrained_clkdivs) {
        BelId option0 = ctx->getBelByName(constr_pair.first);
        BelId option1 = gwu.get_other_hclk_clkdiv2(option0);

        // On the GW1N-9 devices, only the lower CLKDIV can be fed by a CLKDIV2
        std::vector<BelId> options = {option1, option0};
        if (chip.str(ctx) == "GW1N-9C") {
            auto ci = ctx->cells.at(constr_pair.second).get();
            for (auto cluster_child_cell : ci->constr_children)
                if (cluster_child_cell->type == id_CLKDIV2 && options.back() == option0) {
                    options.pop_back();
                    break;
                }
        }

        bool placed = false;
        for (auto option : options) {
            if (placed || (used_bels.find(option) != used_bels.end()))
                continue;
            for (auto option_cell : bel_cell_map[option]) {
                if ((option_cell.first != constr_pair.second) ||
                    (true_clkdivs.find(option_cell.first) != true_clkdivs.end()))
                    continue;
                final_placement[option_cell] = option;
                true_clkdivs[option_cell.first] = option_cell;
                used_bels.insert(option);
                placed = true;
                break;
            }
        }
        // This must be a constrained CLKDIV that either does not serve IOLOGIC Or
        // does not have a direct (HCLK-FCLK) connection IOLOGIC it serves
        // We create a new alias to represent this
        if (!placed) {
            auto new_alias = std::pair<IdString, int>(constr_pair.second, -1);
            for (auto option : options)
                bel_cell_map[option].insert(new_alias);
            alias_cells.push_back(new_alias);
            true_clkdivs[constr_pair.second] = new_alias;
        }
    }

    // Second Matching for CLKDIV2 and routing reservation
    std::map<IdStringList, std::set<std::pair<IdString, int>>> full_hclk_graph;
    for (auto bel_cell_candidates : bel_cell_map) {
        auto bel = bel_cell_candidates.first;
        auto bel_name = ctx->getBelName(bel);
        if (!used_bels.count(bel)) {
            for (auto candidate : bel_cell_candidates.second) {
                if (((candidate.second == -1) || (!true_clkdivs.count(candidate.first)) ||
                     !(true_clkdivs[candidate.first] == candidate))) {
                    full_hclk_graph[bel_name].insert(candidate);
                }
            }
        }
    }

    auto full_matching = gwu.find_maximum_bipartite_matching(full_hclk_graph);
    for (auto belname_cellalias : full_matching) {
        auto bel = ctx->getBelByName(belname_cellalias.first);
        NPNR_ASSERT(!used_bels.count(bel));
        final_placement[belname_cellalias.second] = bel;
    }

    if (ctx->debug) {
        log_info("<-----CUSTOM HCLK PLACER: Second Matching(CLKDIV2 and Routing) Results------>\n");
        for (auto match_pair : full_matching) {
            auto alias = match_pair.second;
            auto bel = match_pair.first;
            auto cell_type = ctx->cells.at(alias.first).get()->type;
            log_info("%s cell %s Alias %d <-----> HCLK Section at %s\n", cell_type.c_str(ctx), alias.first.c_str(ctx),
                     alias.second, bel.str(ctx).c_str());
        }
        log("\n");
    }

    for (auto cell_alias : alias_cells) {
        auto ci = ctx->cells.at(cell_alias.first).get();

        if (final_placement.find(cell_alias) == final_placement.end() && ctx->debug)
            if (ci->type == id_CLKDIV2 || ci->type == id_CLKDIV)
                log_info("Custom HCLK Placer: Unable to place HCLK cell %s; no BELs available to implement cell type "
                         "%s\n",
                         ci->name.c_str(ctx), ci->type.c_str(ctx));
            else
                log_info("Custom HCLK Placer: Unable to guarantee route for HCLK signal from %s to IOLOGIC\n",
                         ci->name.c_str(ctx));

        else {
            auto placement = final_placement[cell_alias];
            if (ctx->debug)
                log_info("Custom HCLK Placer: Placing %s Alias %d at %s\n", cell_alias.first.c_str(ctx),
                         cell_alias.second, ctx->nameOfBel(placement));
            if (ci->type == id_CLKDIV2)
                ctx->bindBel(placement, ci, STRENGTH_LOCKED);

            else if ((ci->type == id_CLKDIV) && (true_clkdivs[cell_alias.first] == cell_alias)) {
                NetInfo *in = ci->getPort(id_HCLKIN);
                if (in && in->driver.cell->type == id_CLKDIV2) {
                    ctx->bindBel(placement, in->driver.cell, STRENGTH_LOCKED);
                }
                auto clkdiv_bel = gwu.get_clkdiv_for_clkdiv2(placement);
                ctx->bindBel(clkdiv_bel, ci, STRENGTH_LOCKED);
            } else {
                if (ctx->debug)
                    log_info("Custom HCLK Placer: Reserving HCLK %s to route clock from %s\n",
                             ctx->nameOfBel(placement), ci->name.c_str(ctx));
                routing_reserved_hclk_sections.insert(placement);
            }
        }
        if (ci->attrs.count(id_BEL))
            ci->unsetAttr(id_BEL);
    }
}

static bool r57_env_enabled(const char *name)
{
    const char *value = getenv(name);
    if (value == nullptr || value[0] == '\0')
        return false;
    return strcmp(value, "0") != 0 && strcmp(value, "false") != 0 && strcmp(value, "FALSE") != 0 &&
           strcmp(value, "off") != 0 && strcmp(value, "OFF") != 0 && strcmp(value, "no") != 0 &&
           strcmp(value, "NO") != 0;
}

static bool r57_env_disabled(const char *name)
{
    const char *value = getenv(name);
    if (value == nullptr || value[0] == '\0')
        return false;
    return strcmp(value, "0") == 0 || strcmp(value, "false") == 0 || strcmp(value, "FALSE") == 0 ||
           strcmp(value, "off") == 0 || strcmp(value, "OFF") == 0 || strcmp(value, "no") == 0 ||
           strcmp(value, "NO") == 0;
}

void GowinImpl::prePlace()
{
    bool r57_debug = r57_env_enabled("R57_PREPLACE_DEBUG");
    if (r57_debug) {
        log_info("R57 prePlace hook ENTER\n");
        log_r57_preplace_debug_ffs();
        log_info("R57 prePlace: before place_constrained_hclk_cells()\n");
    }
    place_constrained_hclk_cells();
    if (r57_debug) {
        log_info("R57 prePlace: after place_constrained_hclk_cells()\n");
        log_info("R57 prePlace: before constrain_r56_fabric_dq_capture_ffs()\n");
    }
    constrain_r56_fabric_dq_capture_ffs();
    if (r57_debug)
        log_info("R57 prePlace: after constrain_r56_fabric_dq_capture_ffs()\n");
    ctx->assignArchInfo();
    assign_cell_info();
    fast_logic_cell.reset(ctx->getGridDimX(), ctx->getGridDimY());
    for (auto bel : ctx->getBels()) {
        if (ctx->getBelType(bel) == id_LUT4) {
            Loc loc = ctx->getBelLocation(bel);
            fast_logic_cell.at(loc.x, loc.y).resize(37);
        }
    }
}

void GowinImpl::prepare_fast_logic_cell()
{
    if (fast_logic_cell.width() == ctx->getGridDimX() && fast_logic_cell.height() == ctx->getGridDimY())
        return;

    fast_logic_cell.reset(ctx->getGridDimX(), ctx->getGridDimY());
    for (auto bel : ctx->getBels()) {
        if (ctx->getBelType(bel) == id_LUT4) {
            Loc loc = ctx->getBelLocation(bel);
            fast_logic_cell.at(loc.x, loc.y).resize(37);
        }
    }
}

struct R56DqCaptureCandidate
{
    int hop;
    int y_delta;
    int x;
    int z;
    BelId bel;
};

static bool r56_tile_xy_in_grid(Context *ctx, int x, int y)
{
    return x >= 0 && y >= 0 && x < ctx->getGridDimX() && y < ctx->getGridDimY();
}

BelId find_r56_nearby_dq_capture_ff(Context *ctx, Loc iob_loc, const GowinImpl *impl, IdString ff_type,
                                     bool r57_debug)
{
    std::vector<R56DqCaptureCandidate> candidates;
    const int max_hop = 4;
    const int grid_x = ctx->getGridDimX();
    const int grid_y = ctx->getGridDimY();
    if (grid_x <= 0 || grid_y <= 0) {
        if (r57_debug)
            log_info("R57 find_r56_nearby_dq_capture_ff: empty grid grid_x=%d grid_y=%d\n", grid_x, grid_y);
        return BelId();
    }
    const int x0 = std::max(0, iob_loc.x - max_hop);
    const int x1 = std::min(grid_x - 1, iob_loc.x + max_hop);
    const int y0 = std::max(0, iob_loc.y - max_hop);
    const int y1 = std::min(grid_y - 1, iob_loc.y + max_hop);
    if (r57_debug) {
        log_info("R57 find_r56_nearby_dq_capture_ff ENTER iob_loc=(%d,%d,%d) grid=(%d,%d) "
                 "scan_x=[%d,%d] scan_y=[%d,%d]\n",
                 iob_loc.x, iob_loc.y, iob_loc.z, grid_x, grid_y, x0, x1, y0, y1);
    }
    if (x0 > x1 || y0 > y1)
        return BelId();
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            if (!r56_tile_xy_in_grid(ctx, x, y)) {
                if (r57_debug)
                    log_info("R57 find_r56_nearby_dq_capture_ff: skip out-of-grid tile x=%d y=%d\n", x, y);
                continue;
            }
            int hop = std::abs(x - iob_loc.x) + std::abs(y - iob_loc.y);
            if (hop > max_hop)
                continue;
            if (r57_debug)
                log_info("R57 find_r56_nearby_dq_capture_ff: before getBelsByTile x=%d y=%d hop=%d\n", x, y,
                         hop);
            for (BelId bel : ctx->getBelsByTile(x, y)) {
                if (r57_debug)
                    log_info("R57 find_r56_nearby_dq_capture_ff: candidate raw_tile=%d raw_index=%d before "
                             "getBelType\n",
                             int(bel.tile), int(bel.index));
                if (ctx->getBelType(bel) != id_DFF)
                    continue;
                if (r57_debug)
                    log_info("R57 find_r56_nearby_dq_capture_ff: DFF candidate raw_tile=%d raw_index=%d before "
                             "checkBelAvail\n",
                             int(bel.tile), int(bel.index));
                if (!ctx->checkBelAvail(bel))
                    continue;
                if (r57_debug)
                    log_info("R57 find_r56_nearby_dq_capture_ff: DFF candidate raw_tile=%d raw_index=%d before "
                             "isValidBelForCellType\n",
                             int(bel.tile), int(bel.index));
                if (!impl->isValidBelForCellType(ff_type, bel))
                    continue;
                if (r57_debug)
                    log_info("R57 find_r56_nearby_dq_capture_ff: DFF candidate raw_tile=%d raw_index=%d before "
                             "getBelLocation(candidate)\n",
                             int(bel.tile), int(bel.index));
                Loc loc = ctx->getBelLocation(bel);
                if (r57_debug)
                    log_info("R57 find_r56_nearby_dq_capture_ff: DFF candidate raw_tile=%d raw_index=%d "
                             "loc=(%d,%d,%d)\n",
                             int(bel.tile), int(bel.index), loc.x, loc.y, loc.z);
                if (!r56_tile_xy_in_grid(ctx, loc.x, loc.y)) {
                    if (r57_debug)
                        log_info("R57 find_r56_nearby_dq_capture_ff: skip candidate outside grid loc=(%d,%d,%d)\n",
                                 loc.x, loc.y, loc.z);
                    continue;
                }
                candidates.push_back(R56DqCaptureCandidate{hop, std::abs(y - iob_loc.y), x, loc.z, bel});
            }
        }
    }
    if (candidates.empty())
        return BelId();
    std::sort(candidates.begin(), candidates.end(), [](const R56DqCaptureCandidate &a, const R56DqCaptureCandidate &b) {
        if (a.hop != b.hop)
            return a.hop < b.hop;
        if (a.y_delta != b.y_delta)
            return a.y_delta < b.y_delta;
        if (a.x != b.x)
            return a.x < b.x;
        return a.z < b.z;
    });
    return candidates.front().bel;
}

static void r56_prepare_fast_logic_cell_for_preplace(Context *ctx,
                                                     array2d<std::vector<CellInfo *>> &fast_logic_cell,
                                                     bool r57_debug)
{
    int grid_x = ctx->getGridDimX();
    int grid_y = ctx->getGridDimY();
    if (fast_logic_cell.width() == grid_x && fast_logic_cell.height() == grid_y)
        return;
    if (r57_debug)
        log_info("R57 constrain_r56_fabric_dq_capture_ffs: reset fast_logic_cell for early bind grid=(%d,%d)\n",
                 grid_x, grid_y);
    fast_logic_cell.reset(grid_x, grid_y);
    for (BelId bel : ctx->getBels()) {
        if (ctx->getBelType(bel) != id_LUT4)
            continue;
        Loc loc = ctx->getBelLocation(bel);
        if (!r56_tile_xy_in_grid(ctx, loc.x, loc.y)) {
            if (r57_debug)
                log_info("R57 constrain_r56_fabric_dq_capture_ffs: skip LUT4 outside grid raw_tile=%d raw_index=%d "
                         "loc=(%d,%d,%d)\n",
                         int(bel.tile), int(bel.index), loc.x, loc.y, loc.z);
            continue;
        }
        fast_logic_cell.at(loc.x, loc.y).resize(37);
    }
}

static bool r56_bel_is_listed(Context *ctx, BelId needle)
{
    if (needle == BelId())
        return false;
    for (BelId bel : ctx->getBels())
        if (bel == needle)
            return true;
    return false;
}

void GowinImpl::log_r57_preplace_debug_ffs(void)
{
    IdString capture_attr = ctx->id("R56_FABRIC_DQ_CAPTURE");
    IdString iob_attr = ctx->id("R56_DQ_IOB_CELL");
    std::map<std::string, CellInfo *> cells_by_name;
    for (auto &cell : ctx->cells) {
        cells_by_name[cell.second->name.str(ctx)] = cell.second.get();
    }

    for (auto &cell : ctx->cells) {
        CellInfo *ff = cell.second.get();
        if (!ff->attrs.count(capture_attr))
            continue;
        std::string iob_name = ff->attrs.count(iob_attr) ? ff->attrs.at(iob_attr).as_string() : std::string("<missing>");
        auto iob_it = cells_by_name.find(iob_name);
        CellInfo *iob = (iob_it == cells_by_name.end()) ? nullptr : iob_it->second;
        bool iob_bel_is_null = (iob == nullptr) || (iob->bel == BelId());
        int raw_tile = (iob == nullptr) ? -1 : int(iob->bel.tile);
        int raw_index = (iob == nullptr) ? -1 : int(iob->bel.index);
        log_info("R57 prePlace tagged FF %s owner_iob=%s owner_found=%d iob_bel_is_BelId=%d "
                 "iob_bel_raw_tile=%d iob_bel_raw_index=%d\n",
                 ctx->nameOf(ff), iob_name.c_str(), iob != nullptr, iob_bel_is_null, raw_tile, raw_index);
    }
}

void GowinImpl::constrain_r56_fabric_dq_capture_ffs(void)
{
    bool place_constrain = !r57_env_disabled("R56_PLACE_CONSTRAIN");
    bool r57_debug = r57_env_enabled("R57_PREPLACE_DEBUG");
    if (!place_constrain && !r57_debug)
        return;
    if (r57_debug)
        log_info("R57 constrain_r56_fabric_dq_capture_ffs: R56_PLACE_CONSTRAIN=%d\n", place_constrain);

    IdString capture_attr = ctx->id("R56_FABRIC_DQ_CAPTURE");
    IdString iob_attr = ctx->id("R56_DQ_IOB_CELL");
    std::map<std::string, CellInfo *> cells_by_name;
    for (auto &cell : ctx->cells) {
        cells_by_name[cell.second->name.str(ctx)] = cell.second.get();
    }

    for (auto &cell : ctx->cells) {
        CellInfo *ff = cell.second.get();
        if (!ff->attrs.count(capture_attr))
            continue;
        if (!is_dff(ff)) {
            log_warning("r56: tagged DQ capture cell %s is no longer a fabric DFF; skipping placement constraint.\n",
                        ctx->nameOf(ff));
            continue;
        }
        if (ff->bel != BelId()) {
            if (ctx->verbose)
                log_info("r56: tagged DQ capture FF %s is already placed; skipping placement constraint.\n",
                         ctx->nameOf(ff));
            continue;
        }
        if (!ff->attrs.count(iob_attr)) {
            log_warning("r56: tagged DQ capture FF %s has no owning IOB attribute; skipping placement constraint.\n",
                        ctx->nameOf(ff));
            continue;
        }
        std::string iob_name = ff->attrs.at(iob_attr).as_string();
        auto iob_it = cells_by_name.find(iob_name);
        if (iob_it == cells_by_name.end()) {
            log_warning("r56: tagged DQ capture FF %s references missing IOB %s; skipping placement constraint.\n",
                        ctx->nameOf(ff), iob_name.c_str());
            continue;
        }
        CellInfo *iob = iob_it->second;
        if (iob->bel == BelId()) {
            log_warning("r56: tagged DQ capture FF %s owner %s has no BEL in prePlace; skipping placement constraint.\n",
                        ctx->nameOf(ff), ctx->nameOf(iob));
            continue;
        }
        if (!place_constrain) {
            if (r57_debug)
                log_info("R57: R56_PLACE_CONSTRAIN is off; not constraining tagged DQ capture FF %s.\n",
                         ctx->nameOf(ff));
            continue;
        }
        if (!r56_bel_is_listed(ctx, iob->bel)) {
            log_warning("r56: tagged DQ capture FF %s owner %s has a non-null BEL that is not in ctx->getBels(); "
                        "skipping placement constraint.\n",
                        ctx->nameOf(ff), ctx->nameOf(iob));
            continue;
        }

        if (r57_debug)
            log_info("R57 constrain_r56_fabric_dq_capture_ffs: FF %s owner %s before getBelLocation(iob->bel) "
                     "raw_tile=%d raw_index=%d\n",
                     ctx->nameOf(ff), ctx->nameOf(iob), int(iob->bel.tile), int(iob->bel.index));
        Loc iob_loc = ctx->getBelLocation(iob->bel);
        if (r57_debug)
            log_info("R57 constrain_r56_fabric_dq_capture_ffs: FF %s owner %s iob_loc=(%d,%d,%d)\n",
                     ctx->nameOf(ff), ctx->nameOf(iob), iob_loc.x, iob_loc.y, iob_loc.z);
        if (!r56_tile_xy_in_grid(ctx, iob_loc.x, iob_loc.y)) {
            log_warning("r56: tagged DQ capture FF %s owner %s has out-of-grid BEL location (%d,%d,%d); "
                        "skipping placement constraint.\n",
                        ctx->nameOf(ff), ctx->nameOf(iob), iob_loc.x, iob_loc.y, iob_loc.z);
            continue;
        }

        r56_prepare_fast_logic_cell_for_preplace(ctx, fast_logic_cell, r57_debug);
        if (r57_debug)
            log_info("R57 constrain_r56_fabric_dq_capture_ffs: before find_r56_nearby_dq_capture_ff for FF %s "
                     "iob_loc=(%d,%d,%d)\n",
                     ctx->nameOf(ff), iob_loc.x, iob_loc.y, iob_loc.z);
        BelId bel = find_r56_nearby_dq_capture_ff(ctx, iob_loc, this, ff->type, r57_debug);
        if (bel == BelId()) {
            log_warning("r56: unable to find a nearby fabric DFF BEL for DQ capture FF %s at %s; "
                        "leaving it to the normal placer.\n",
                        ctx->nameOf(ff), ctx->nameOf(iob));
            continue;
        }

        if (r57_debug)
            log_info("R57 constrain_r56_fabric_dq_capture_ffs: selected raw_tile=%d raw_index=%d before "
                     "getBelLocation(selected)\n",
                     int(bel.tile), int(bel.index));
        Loc ff_loc = ctx->getBelLocation(bel);
        if (!r56_tile_xy_in_grid(ctx, ff_loc.x, ff_loc.y)) {
            log_warning("r56: selected DQ capture BEL for %s has out-of-grid location (%d,%d,%d); "
                        "leaving it to the normal placer.\n",
                        ctx->nameOf(ff), ff_loc.x, ff_loc.y, ff_loc.z);
            continue;
        }
        if (int(fast_logic_cell.at(ff_loc.x, ff_loc.y).size()) <= ff_loc.z)
            fast_logic_cell.at(ff_loc.x, ff_loc.y).resize(std::max(37, ff_loc.z + 1));
        int hop = std::abs(ff_loc.x - iob_loc.x) + std::abs(ff_loc.y - iob_loc.y);
        if (r57_debug)
            log_info("R57 constrain_r56_fabric_dq_capture_ffs: before bindBel raw_tile=%d raw_index=%d "
                     "ff_loc=(%d,%d,%d) hop=%d\n",
                     int(bel.tile), int(bel.index), ff_loc.x, ff_loc.y, ff_loc.z, hop);
        ctx->bindBel(bel, ff, PlaceStrength::STRENGTH_LOCKED);
        ff->setAttr(ctx->id("R56_DQ_IOB_BEL"), Property(ctx->getBelName(iob->bel).str(ctx)));
        ff->setAttr(ctx->id("R56_PAD_TO_FF_TILE_HOPS"), hop);
        log_info("  r56: locked DQ capture FF %s at %s near IOB %s (tile_hops=%d)\n", ctx->nameOf(ff),
                 ctx->nameOfBel(bel), ctx->nameOfBel(iob->bel), hop);
    }
}

void GowinImpl::postPlace()
{
    log_info("[Codex round 12] SEL[i] rejection count during placement: %llu\n",
             (unsigned long long)g_sel_i_reject_count.load(std::memory_order_relaxed));

    if (ctx->debug) {
        log_info("================== Final Placement ===================\n");
        for (auto &cell : ctx->cells) {
            auto ci = cell.second.get();
            if (ci->bel != BelId()) {
                log_info("%s: %s\n", ctx->nameOfBel(ci->bel), ctx->nameOf(ci));
            } else {
                log_info("unknown: %s\n", ctx->nameOf(ci));
            }
        }
        log_break();
    }

    // adjust cell pin to bel pin mapping for DSP cells (CE, CLK and RESET pins)
    adjust_dsp_pin_mapping();
    create_passthrough_luts();
}

void GowinImpl::preRoute() { gowin_route_globals(ctx); }

void GowinImpl::postRoute()
{
    std::set<IdString> visited_hclk_users;

    for (auto &cell : ctx->cells) {
        auto ci = cell.second.get();
        if (ci->type.in(id_IOLOGICI, id_IOLOGICO, id_IOLOGIC) ||
            ((is_iologici(ci) || is_iologico(ci)) && !ci->type.in(id_ODDR, id_ODDRC, id_IDDR, id_IDDRC))) {
            if (visited_hclk_users.find(ci->name) == visited_hclk_users.end()) {
                // mark FCLK<-HCLK connections
                const NetInfo *h_net = ci->getPort(id_FCLK);
                if (h_net) {
                    for (auto &user : h_net->users) {
                        if (user.port != id_FCLK) {
                            continue;
                        }
                        user.cell->setAttr(id_IOLOGIC_FCLK, Property("UNKNOWN"));
                        visited_hclk_users.insert(user.cell->name);
                        PipId up_pip = h_net->wires.at(ctx->getNetinfoSinkWire(h_net, user, 0)).pip;
                        IdString up_wire_name = ctx->getWireName(ctx->getPipSrcWire(up_pip))[1];
                        if (!gwu.has_5A_HCLK()) {
                            if (up_wire_name.in(id_HCLK_OUT0, id_HCLK_OUT1, id_HCLK_OUT2, id_HCLK_OUT3)) {
                                user.cell->setAttr(id_IOLOGIC_FCLK, Property(up_wire_name.str(ctx)));
                                if (ctx->debug) {
                                    log_info("set IOLOGIC_FCLK to %s\n", up_wire_name.c_str(ctx));
                                }
                            }
                        } else if (up_wire_name.in(id_HCLK00, id_HCLK10, id_HCLK20, id_HCLK30)) {
                            user.cell->setAttr(id_IOLOGIC_FCLK, Property("HCLK_OUT0"));
                        } else if (up_wire_name.in(id_HCLK01, id_HCLK11, id_HCLK21, id_HCLK31)) {
                            user.cell->setAttr(id_IOLOGIC_FCLK, Property("HCLK_OUT1"));
                        } else if (up_wire_name.in(id_HCLK02, id_HCLK12, id_HCLK22, id_HCLK32)) {
                            user.cell->setAttr(id_IOLOGIC_FCLK, Property("HCLK_OUT2"));
                        } else if (up_wire_name.in(id_HCLK03, id_HCLK13, id_HCLK23, id_HCLK33)) {
                            user.cell->setAttr(id_IOLOGIC_FCLK, Property("HCLK_OUT3"));
                        }
                        if (ctx->debug) {
                            log_info("HCLK user cell:%s, port:%s, wire:%s, pip:%s, up wire:%s\n",
                                     ctx->nameOf(user.cell), user.port.c_str(ctx),
                                     ctx->nameOfWire(ctx->getNetinfoSinkWire(h_net, user, 0)), ctx->nameOfPip(up_pip),
                                     ctx->nameOfWire(ctx->getPipSrcWire(up_pip)));
                        }
                    }
                }
            }
        } else {
            if (is_pll(ci)) {
                // CLKIN is connected to HLCK?
                NetInfo *h_net = ci->getPort(id_CLKIN);
                if (h_net == nullptr || h_net->wires.empty()) {
                    continue;
                }
                PortRef pr = {ci, id_CLKIN};
                PipId up_pip = h_net->wires.at(ctx->getNetinfoSinkWire(h_net, pr, 0)).pip;
                IdString up_wire_name = ctx->getWireName(ctx->getPipSrcWire(up_pip))[1];
                if (up_wire_name.in(id_HCLK_OUT0, id_HCLK_OUT1)) {
                    ci->setParam(id_INSEL, Property("CLKIN3"));
                } else {
                    if (up_wire_name.in(id_HCLK_OUT2, id_HCLK_OUT3)) {
                        ci->setParam(id_INSEL, Property("CLKIN4"));
                    }
                }
                // CLKFB is connected to HLCK?
                h_net = ci->getPort(id_CLKFB);
                if (h_net == nullptr || h_net->wires.empty()) {
                    continue;
                }
                pr.port = id_CLKFB;
                up_pip = h_net->wires.at(ctx->getNetinfoSinkWire(h_net, pr, 0)).pip;
                up_wire_name = ctx->getWireName(ctx->getPipSrcWire(up_pip))[1];
                if (up_wire_name.in(id_HCLK_OUT0, id_HCLK_OUT1)) {
                    ci->setParam(id_FBSEL, Property("CLKFB1"));
                } else {
                    if (up_wire_name.in(id_HCLK_OUT2, id_HCLK_OUT3)) {
                        ci->setParam(id_FBSEL, Property("CLKFB4"));
                    }
                }
            }
        }
    }
    std::vector<CellInfo *> to_remove;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type.in(id_BLOCKER_LUT, id_BLOCKER_FF)) {
            to_remove.push_back(ci);
        }
    }
    for (auto ci : to_remove) {
        auto root = ctx->cells.at(ci->cluster).get();
        root->constr_children.erase(std::remove_if(root->constr_children.begin(), root->constr_children.end(),
                                                   [&](CellInfo *c) { return c == ci; }));
        ctx->cells.erase(ci->name);
    }
}

bool GowinImpl::isBelLocationValid(BelId bel, bool explain_invalid) const
{
    Loc l = ctx->getBelLocation(bel);
    IdString bel_type = ctx->getBelType(bel);
    if (!ctx->getBoundBelCell(bel)) {
        return true;
    }
    switch (bel_type.hash()) {
    case ID_LUT4: /* fall-through */
    case ID_DFF:
        return slice_valid(l.x, l.y, l.z / 2);
    case ID_ALU:
        return slice_valid(l.x, l.y, l.z - BelZ::ALU0_Z);
    case ID_RAM16SDP4:
        return slice_valid(l.x, l.y, 0);
    case ID_MUX2_LUT5:
        return slice_valid(l.x, l.y, (l.z - BelZ::MUX20_Z) / 2);
    case ID_MUX2_LUT6:
        return slice_valid(l.x, l.y, (l.z - BelZ::MUX21_Z) / 2 + 1);
    case ID_MUX2_LUT7:
        return slice_valid(l.x, l.y, 3);
    case ID_MUX2_LUT8:
        return slice_valid(l.x, l.y, 7);
    case ID_PADD9:           /* fall-through */
    case ID_PADD18:          /* fall-through */
    case ID_MULT9X9:         /* fall-through */
    case ID_MULT12X12:       /* fall-through */
    case ID_MULT18X18:       /* fall-through */
    case ID_MULTADDALU18X18: /* fall-through */
    case ID_MULTALU18X18:    /* fall-through */
    case ID_MULTALU36X18:    /* fall-through */
    case ID_MULT36X36:       /* fall-through */
    case ID_ALU54D:
        return dsp_valid(l, bel_type, explain_invalid);
    case ID_CLKDIV2: /* fall-through */
    case ID_CLKDIV:
        if (gwu.has_5A_HCLK()) {
            return true;
        }
        return hclk_valid(bel, bel_type);
    }
    return true;
}

// Bel bucket functions
IdString GowinImpl::getBelBucketForCellType(IdString cell_type) const
{
    if (cell_type.in(id_IBUF, id_OBUF)) {
        return id_IOB;
    }
    if (cell_type.in(id_MIPI_OBUF, id_MIPI_OBUF_A)) {
        return id_MIPI_OBUF;
    }
    if (type_is_lut(cell_type) || cell_type == id_BLOCKER_LUT) {
        return id_LUT4;
    }
    if (type_is_dff(cell_type) || cell_type == id_BLOCKER_FF) {
        return id_DFF;
    }
    if (type_is_ssram(cell_type)) {
        return id_RAM16SDP4;
    }
    if (type_is_iologici(cell_type)) {
        return id_IOLOGICI;
    }
    if (type_is_iologico(cell_type)) {
        return id_IOLOGICO;
    }
    if (type_is_bsram(cell_type)) {
        return id_BSRAM;
    }
    if (cell_type == id_GOWIN_GND) {
        return id_GND;
    }
    if (cell_type == id_GOWIN_VCC) {
        return id_VCC;
    }
    return cell_type;
}

bool GowinImpl::isValidBelForCellType(IdString cell_type, BelId bel) const
{
    if (cell_type == id_DUMMY_CELL) {
        return true;
    }

    IdString bel_type = ctx->getBelType(bel);
    if (bel_type == id_IOB) {
        return cell_type.in(id_IBUF, id_OBUF);
    }
    if (bel_type == id_MIPI_OBUF) {
        return cell_type.in(id_MIPI_OBUF, id_MIPI_OBUF_A);
    }
    if (bel_type == id_LUT4) {
        return type_is_lut(cell_type) || cell_type == id_BLOCKER_LUT;
    }
    if (bel_type == id_DFF) {
        return type_is_dff(cell_type) || cell_type == id_BLOCKER_FF;
    }
    if (bel_type == id_RAM16SDP4) {
        return type_is_ssram(cell_type);
    }
    if (bel_type == id_IOLOGICI) {
        return type_is_iologici(cell_type);
    }
    if (bel_type == id_IOLOGICO) {
        return type_is_iologico(cell_type);
    }
    if (bel_type == id_BSRAM) {
        return type_is_bsram(cell_type);
    }
    if (bel_type == id_GND) {
        return cell_type == id_GOWIN_GND;
    }
    if (bel_type == id_VCC) {
        return cell_type == id_GOWIN_VCC;
    }
    return (bel_type == cell_type);
}

void GowinImpl::assign_cell_info()
{
    fast_cell_info.resize(ctx->cells.size());
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        auto &fc = fast_cell_info.at(ci->flat_index);
        if (is_lut(ci)) {
            fc.lut_f = ci->getPort(id_F);
            continue;
        }
        if (is_dff(ci)) {
            fc.ff_d = ci->getPort(id_D);
            fc.ff_clk = ci->getPort(id_CLK);
            fc.ff_ce = ci->getPort(id_CE);
            for (IdString port : {id_SET, id_RESET, id_PRESET, id_CLEAR}) {
                fc.ff_lsr = ci->getPort(port);
                if (fc.ff_lsr != nullptr) {
                    break;
                }
            }
            continue;
        }
        if (is_alu(ci)) {
            fc.alu_sum = ci->getPort(id_SUM);
            continue;
        }
        auto get_net = [&](IdString port_id) {
            NetInfo *ni = ci->getPort(port_id);
            if (ni != nullptr && ni->driver.cell == nullptr) {
                ni = nullptr;
            }
            return ni;
        };
        if (is_dsp(ci)) {
            fc.dsp_reset = get_net(id_RESET);
            fc.dsp_clk = get_net(id_CLK);
            fc.dsp_ce = get_net(id_CE);
            fc.dsp_asign = get_net(id_ASIGN);
            fc.dsp_bsign = get_net(id_BSIGN);
            fc.dsp_asel = get_net(id_ASEL);
            fc.dsp_bsel = get_net(id_BSEL);
            fc.dsp_soa_reg = ci->params.count(id_SOA_REG) && ci->params.at(id_SOA_REG).as_int64() == 1;
            fc.dsp_5a_clk0 = get_net(id_CLK0);
            fc.dsp_5a_clk1 = get_net(id_CLK1);
            fc.dsp_5a_ce0 = get_net(id_CE0);
            fc.dsp_5a_ce1 = get_net(id_CE1);
            fc.dsp_5a_reset0 = get_net(id_RESET0);
            fc.dsp_5a_reset1 = get_net(id_RESET1);
        }
    }
}

// If there is an unused LUT next to the DFF, use its inputs for the D input
void GowinImpl::create_passthrough_luts(void)
{
    std::vector<std::unique_ptr<CellInfo>> new_cells;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (is_dff(ci)) {
            Loc loc = ctx->getBelLocation(ci->bel);
            BelId lut_bel = ctx->getBelByLocation(Loc(loc.x, loc.y, loc.z - 1));
            CellInfo *lut = ctx->getBoundBelCell(lut_bel);
            CellInfo *alu = ctx->getBoundBelCell(ctx->getBelByLocation(Loc(loc.x, loc.y, loc.z / 2 + BelZ::ALU0_Z)));
            const CellInfo *ramw = ctx->getBoundBelCell(ctx->getBelByLocation(Loc(loc.x, loc.y, BelZ::RAMW_Z)));

            if (!(lut || alu || ramw)) {
                if (ctx->debug) {
                    log_info("Found an unused LUT:%s, ", ctx->nameOfBel(lut_bel));
                }
                // make LUT
                auto lut_cell = gwu.create_cell(gwu.create_aux_name(ci->name, 0, "_passthrough_lut$"), id_LUT4);
                CellInfo *lut = lut_cell.get();
                NetInfo *d_net = ci->getPort(id_D);
                NPNR_ASSERT(d_net != nullptr);

                if (d_net->name == ctx->id("$PACKER_GND") || d_net->name == ctx->id("$PACKER_VCC")) {
                    if (ctx->debug) {
                        log_info("make a constant %s.\n", d_net->name == ctx->id("$PACKER_VCC") ? "VCC" : "GND");
                    }
                    ci->disconnectPort(id_D);
                    if (d_net->name == ctx->id("$PACKER_GND")) {
                        lut->setParam(id_INIT, 0x0000);
                    } else {
                        lut->setParam(id_INIT, 0xffff);
                    }
                } else {
                    if (ctx->debug) {
                        log_info("make a pass-through.\n");
                    }
                    IdString lut_input = id_I3;
                    int lut_init = 0xff00;

                    lut->addInput(lut_input);
                    lut->cell_bel_pins[lut_input].clear();
                    lut->cell_bel_pins.at(lut_input).push_back(lut_input);
                    ci->movePortTo(id_D, lut, lut_input);
                    lut->setParam(id_INIT, lut_init);
                }
                lut->addOutput(id_F);
                lut->cell_bel_pins[id_F].clear();
                lut->cell_bel_pins.at(id_F).push_back(id_F);
                ci->connectPorts(id_D, lut, id_F);

                ctx->bindBel(lut_bel, lut, PlaceStrength::STRENGTH_LOCKED);
                new_cells.push_back(std::move(lut_cell));
            }
        }
    }
    for (auto &cell : new_cells) {
        ctx->cells[cell->name] = std::move(cell);
    }
}

// DFFs must have compatible LSR kind. Round 24 fix:
// GW5A slice has ONE REGSET fuse selecting RESET vs SET vs CLEAR vs PRESET.
// Two FFs in the same slice pair can only share that fuse if their LSR kinds
// match (or one of them has no LSR at all). Previous logic explicitly allowed
// RESET+SET and CLEAR+PRESET pairs, which is physically impossible: the
// hardware can encode one polarity, not both. Result was that one FF per
// such pair came up in the wrong reset state, scattered across the design,
// breaking boot at scale (P15B etc.) while small designs (top_oled_direct)
// happened to dodge it because the placer never collided incompatible kinds.
enum FfLsrKind { LSR_NONE_KIND, LSR_RESET_KIND, LSR_SET_KIND, LSR_CLEAR_KIND, LSR_PRESET_KIND };

static FfLsrKind ff_lsr_kind(IdString t)
{
    if (t.in(id_DFFR, id_DFFRE, id_DFFNR, id_DFFNRE)) return LSR_RESET_KIND;
    if (t.in(id_DFFS, id_DFFSE, id_DFFNS, id_DFFNSE)) return LSR_SET_KIND;
    if (t.in(id_DFFC, id_DFFCE, id_DFFNC, id_DFFNCE)) return LSR_CLEAR_KIND;
    if (t.in(id_DFFP, id_DFFPE, id_DFFNP, id_DFFNPE)) return LSR_PRESET_KIND;
    return LSR_NONE_KIND;
}

inline bool incompatible_ffs(const CellInfo *ff, const CellInfo *adj_ff)
{
    FfLsrKind a = ff_lsr_kind(ff->type);
    FfLsrKind b = ff_lsr_kind(adj_ff->type);
    if (a != LSR_NONE_KIND && b != LSR_NONE_KIND && a != b)
        return true;
    return false;
}

// placement validation
bool GowinImpl::dsp_valid(Loc l, IdString bel_type, bool explain_invalid) const
{
    const CellInfo *dsp = ctx->getBoundBelCell(ctx->getBelByLocation(l));
    const auto &dsp_data = fast_cell_info.at(dsp->flat_index);

    BelId dsp_macro_bel = ctx->getBelByLocation(Loc(l.x, l.y, gwu.get_dsp_macro(l.z)));
    if (dsp_info.count(dsp_macro_bel)) {
        if (dsp_info.at(dsp_macro_bel).mode9bit && dsp_info.at(dsp_macro_bel).mode18bit) {
            if (explain_invalid) {
                log_nonfatal_error("Different operand lengths (9 and 18) are not permitted in one DSP macro.\n");
            }
            return false;
        }
    }

    // check for shift out register - there is only one for macro
    if (dsp_data.dsp_soa_reg) {
        if (l.z == BelZ::MULT18X18_0_1_Z || l.z == BelZ::MULT18X18_1_1_Z || l.z == BelZ::MULT9X9_0_0_Z ||
            l.z == BelZ::MULT9X9_0_1_Z || l.z == BelZ::MULT9X9_1_0_Z || l.z == BelZ::MULT9X9_1_1_Z) {
            if (explain_invalid) {
                log_nonfatal_error(
                        "It is not possible to place the DSP so that the SOA register is on the macro boundary.\n");
            }
            return false;
        }
    }

    if (bel_type.in(id_MULT9X9, id_PADD9)) {
        int pair_z = gwu.get_dsp_paired_9(l.z);
        const CellInfo *adj_dsp9 = ctx->getBoundBelCell(ctx->getBelByLocation(Loc(l.x, l.y, pair_z)));
        if (adj_dsp9 != nullptr) {
            const auto &adj_dsp9_data = fast_cell_info.at(adj_dsp9->flat_index);
            if ((dsp_data.dsp_asign != adj_dsp9_data.dsp_asign) || (dsp_data.dsp_bsign != adj_dsp9_data.dsp_bsign) ||
                (dsp_data.dsp_asel != adj_dsp9_data.dsp_asel) || (dsp_data.dsp_bsel != adj_dsp9_data.dsp_bsel) ||
                (dsp_data.dsp_reset != adj_dsp9_data.dsp_reset) || (dsp_data.dsp_ce != adj_dsp9_data.dsp_ce) ||
                (dsp_data.dsp_clk != adj_dsp9_data.dsp_clk)) {
                if (explain_invalid) {
                    log_nonfatal_error("For 9bit primitives the control signals must be same.\n");
                }
                return false;
            }
        }
    }

    if (bel_type == id_MULT12X12) {
        int pair_z = gwu.get_dsp_paired_12(l.z);
        const CellInfo *adj_dsp12 = ctx->getBoundBelCell(ctx->getBelByLocation(Loc(l.x, l.y, pair_z)));
        if (adj_dsp12 != nullptr) {
            const auto &adj_dsp12_data = fast_cell_info.at(adj_dsp12->flat_index);
            if ((dsp_data.dsp_5a_clk0 != adj_dsp12_data.dsp_5a_clk0) ||
                (dsp_data.dsp_5a_clk1 != adj_dsp12_data.dsp_5a_clk1) ||
                (dsp_data.dsp_5a_ce0 != adj_dsp12_data.dsp_5a_ce0) ||
                (dsp_data.dsp_5a_ce1 != adj_dsp12_data.dsp_5a_ce1) ||
                (dsp_data.dsp_5a_reset0 != adj_dsp12_data.dsp_5a_reset0) ||
                (dsp_data.dsp_5a_reset1 != adj_dsp12_data.dsp_5a_reset1)) {
                if (explain_invalid) {
                    log_nonfatal_error("For MULT12X12 primitives the control signals must be same.\n");
                }
                return false;
            }
        }
    }

    // check for control nets "overflow"
    BelId dsp_bel = ctx->getBelByLocation(Loc(l.x, l.y, BelZ::DSP_Z));
    if (dsp_info.count(dsp_bel)) {
        if (dsp_info.at(dsp_bel).reset.size() > 4) {
            if (explain_invalid) {
                log_nonfatal_error("More than 4 different networks for RESET signals in one DSP are not allowed.\n");
            }
            return false;
        }
    }
    if (dsp_info.count(dsp_macro_bel)) {
        if (dsp_info.at(dsp_macro_bel).ce.size() > 4 || dsp_info.at(dsp_macro_bel).clk.size() > 4) {
            if (explain_invalid) {
                log_nonfatal_error(
                        "More than 4 different networks for CE or CLK signals in one DSP macro are not allowed.\n");
            }
            return false;
        }
    }
    return true;
}

bool GowinImpl::slice_valid(int x, int y, int z) const
{
    auto &bels = fast_logic_cell.at(x, y);
    const CellInfo *lut = bels.at(z * 2);
    const CellInfo *ff = bels.at(z * 2 + 1);

    // Phase 7e: GSR cell's GSRI port maps to LSR0 wire of its tile.
    // FFs in slots 0,1 share LSR0 via R/S/C/P ports. If GSR is in this
    // tile, reject any FF placement at slot 0 or 1 to avoid router LSR0
    // conflict ($PACKER_VCC from GSR.GSRI vs $PACKER_GND from FF.R).
    if (ff && ff->type != id_BLOCKER_FF && (z == 0 || z == 1)) {
        for (BelId b : ctx->getBelsByTile(x, y)) {
            if (ctx->getBelType(b) != id_GSR) continue;
            if (ctx->getBoundBelCell(b) != nullptr) {
                return false;
            }
        }
    }
    // There are only 6 ALUs
    const CellInfo *alu = (z < 6) ? bels.at(z + BelZ::ALU0_Z) : nullptr;
    const CellInfo *ramw = bels.at(BelZ::RAMW_Z);

    auto is_not_blocker = [](const CellInfo *ci) { return ci && !ci->type.in(id_BLOCKER_LUT, id_BLOCKER_FF); };

    if (alu && lut && lut->type != id_BLOCKER_LUT) {
        return false;
    }

    if (ramw) {
        // FFs in slices 4 and 5 are not allowed
        // also temporarily disallow FF to be placed near RAM
        if (is_not_blocker(bels.at(0 * 2 + 1)) || is_not_blocker(bels.at(1 * 2 + 1)) ||
            is_not_blocker(bels.at(2 * 2 + 1)) || is_not_blocker(bels.at(3 * 2 + 1)) ||
            is_not_blocker(bels.at(4 * 2 + 1)) || is_not_blocker(bels.at(5 * 2 + 1))) {
            return false;
        }
        if (gwu.has_DFF67()) {
            if (is_not_blocker(bels.at(6 * 2 + 1)) || is_not_blocker(bels.at(7 * 2 + 1))) {
                return false;
            }
        }
        // ALU/LUTs in slices 4, 5, 6, 7 are not allowed
        for (int i = 4; i < 8; ++i) {
            if (is_not_blocker(bels.at(i * 2))) {
                return false;
            }
            if (i < 6 && bels.at(i + BelZ::ALU0_Z)) {
                return false;
            }
        }
    }

    // check for ALU/LUT in the adjacent cell
    int adj_lut_z = (1 - (z & 1) * 2 + z) * 2;
    int adj_alu_z = adj_lut_z / 2 + BelZ::ALU0_Z;
    const CellInfo *adj_lut = bels.at(adj_lut_z);
    const CellInfo *adj_ff = bels.at(adj_lut_z + 1);
    const CellInfo *adj_alu = adj_alu_z < (6 + BelZ::ALU0_Z) ? bels.at(adj_alu_z) : nullptr;

    if ((alu && ((adj_lut && adj_lut->type != id_BLOCKER_LUT) || (adj_ff && !adj_alu))) ||
        (((lut && lut->type != id_BLOCKER_LUT) || (ff && !alu)) && adj_alu)) {
        return false;
    }

    if (ff && ff->type != id_BLOCKER_FF) {
        static std::vector<int> mux_z = {BelZ::MUX20_Z,     BelZ::MUX21_Z,     BelZ::MUX20_Z + 4,  BelZ::MUX23_Z,
                                         BelZ::MUX20_Z + 8, BelZ::MUX21_Z + 8, BelZ::MUX20_Z + 12, BelZ::MUX27_Z};
        const auto &ff_data = fast_cell_info.at(ff->flat_index);
        const NetInfo *src;
        // check implcit LUT(ALU) -> FF connection
        NPNR_ASSERT(!ramw); // XXX shouldn't happen for now
        if (alu) {
            // ALU-driven FF: must use ALU sum directly (no REG_SD path).
            src = fast_cell_info.at(alu->flat_index).alu_sum;
            if (ff_data.ff_d != src) {
                return false;
            }
        } else if (lut && lut->type != id_BLOCKER_LUT) {
            // GW5A REG_SD relaxation (Phase 5, restored):
            // Allow LUT + unrelated FF in same slot. FF takes data via SEL[z]
            // route (REG_SD bit = 1). Companion fix: canonicalize_inactive_lsr_ports
            // (rewires inactive VCC-on-LSR to GND so router doesnt see two
            // nets driving same LSR0 wire).
            src = fast_cell_info.at(lut->flat_index).lut_f;
            // SEL[z] mux/FF-D conflict check (Codex round 10, 2026-05-05):
            // SEL[z] is physically shared between wide-LUT mux S0 input and
            // FF-D alternate routing path SEL[z] -> XD[z]. If FF at slice z
            // takes data via the SEL[z]->XD[z] alternate path (ff.D != lut.F),
            // and a wide mux at this tile uses SEL[z] for a *different* net,
            // the placement is physically illegal. router2 catches it as
            // "attempting to reserve sink input path wire .../SEL0 for nets X and Y".
            if (ff_data.ff_d != src) {
                BelId mux_bel = ctx->getBelByLocation(Loc(x, y, mux_z.at(z)));
                CellInfo *mux_cell = ctx->getBoundBelCell(mux_bel);
                if (mux_cell != nullptr &&
                    (mux_cell->type == id_MUX2_LUT5 || mux_cell->type == id_MUX2_LUT6 ||
                     mux_cell->type == id_MUX2_LUT7 || mux_cell->type == id_MUX2_LUT8)) {
                    NetInfo *mux_sel = mux_cell->getPort(id_S0);
                    if (mux_sel != nullptr && mux_sel != ff_data.ff_d) {
                        g_sel_i_reject_count.fetch_add(1, std::memory_order_relaxed);
                        return false;
                    }
                }
            }
        }
        if (adj_ff) {
            if (incompatible_ffs(ff, adj_ff)) {
                return false;
            }

            // Phase 7d (revert canon): ORIGINAL strict LSR check restored.
            // The router refuses to merge $PACKER_GND and $PACKER_VCC arcs
            // both targeting LSR0, so the placer must NOT place adj FFs
            // with mismatched LSR nets even if both are "semantically inactive".
            // Strict equality is the only safe rule.
            const auto &adj_ff_data = fast_cell_info.at(adj_ff->flat_index);
            if (adj_ff_data.ff_lsr != ff_data.ff_lsr) {
                return false;
            }
            if (adj_ff_data.ff_clk != ff_data.ff_clk) {
                return false;
            }
            if (adj_ff_data.ff_ce != ff_data.ff_ce) {
                return false;
            }
        }
        // Check whether the current architecture allows 6 and 7 DFFs
        if (z > 3 && gwu.has_DFF67()) {
            // The 4th, 5th, 6th, and 7th DFFs have the same control wires. Let's check this.
            const int adj_top_ff_z = (5 - (z >> 1)) * 4 + 1;
            for (int i = 0; i < 4; i += 2) {
                const CellInfo *adj_top_ff = bels.at(adj_top_ff_z + i);
                if (adj_top_ff) {
                    const auto &adj_top_ff_data = fast_cell_info.at(adj_top_ff->flat_index);
                    if (adj_top_ff_data.ff_lsr != ff_data.ff_lsr) {
                        return false;
                    }
                    if (adj_top_ff_data.ff_clk != ff_data.ff_clk) {
                        return false;
                    }
                    if (adj_top_ff_data.ff_ce != ff_data.ff_ce) {
                        return false;
                    }
                    // It is sufficient to check only one DFF.
                    break;
                }
            }
        }
    }
    return true;
}
/*
    Every HCLK section can be used in one of 3 ways:
        1. As a simple routing path to IOLOGIC FCLK
        2. As a CLKDIV2
        3. As a CLKDIV (potentially fed by the CLKDIV2 in its section)

    Here we validate that the placement of cells fits within these 3 use cases, while ensuring that
    we enforce the constraint that only 1 CLKDIV can be used per HCLK (there is only 1 CLKDIV in each
    HCLK but we pretend there are two because doing so makes it easier to enforce the real constraint
    that HCLK signals don't crisscross between HCLK sections even after "transformation" by a CLKDIV
    or CLKDIV2)
*/
bool GowinImpl::hclk_valid(BelId bel, IdString bel_type) const
{
    if (bel_type == id_CLKDIV2) {
        if (routing_reserved_hclk_sections.count(bel))
            return false;
        auto clkdiv_cell = ctx->getBoundBelCell(gwu.get_clkdiv_for_clkdiv2(bel));
        if (clkdiv_cell && ctx->getBoundBelCell(bel)->cluster != clkdiv_cell->name)
            return false;
        return true;
    } else if (bel_type == id_CLKDIV) {
        BelId clkdiv2_bel = gwu.get_clkdiv2_for_clkdiv(bel);
        if (routing_reserved_hclk_sections.count(clkdiv2_bel)) {
            return false;
        }

        auto other_clkdiv_cell = ctx->getBoundBelCell(gwu.get_other_hclk_clkdiv(bel));
        if (other_clkdiv_cell)
            return false;

        auto clkdiv2_bel_cell = ctx->getBoundBelCell(clkdiv2_bel);
        if (clkdiv2_bel_cell && clkdiv2_bel_cell->cluster != ctx->getBoundBelCell(bel)->name)
            return false;

        if (clkdiv2_bel_cell && chip.str(ctx) == "GW1N-9C") {
            // On the GW1N(R)-9C, it appears that only the 'odd' CLKDIV2 is connected to CLKDIV
            Loc loc = ctx->getBelLocation(bel);
            if (loc.z == BelZ::CLKDIV_0_Z || loc.z == BelZ::CLKDIV_2_Z)
                return false;
        }

        return true;
    }
    return false;
}

// Cluster
bool GowinImpl::getClusterPlacement(ClusterId cluster, BelId root_bel,
                                    std::vector<std::pair<CellInfo *, BelId>> &placement) const
{
    CellInfo *root_ci = getClusterRootCell(cluster);
    if (!root_ci->type.in(id_PADD9, id_MULT9X9, id_PADD18, id_MULT18X18, id_MULTALU18X18, id_MULTALU36X18,
                          id_MULTADDALU18X18, id_ALU54D, id_MULTADDALU12X12, id_MULTALU27X18)) {
        return HimbaechelAPI::getClusterPlacement(cluster, root_bel, placement);
    }

    NPNR_ASSERT(root_bel != BelId());
    if (!isValidBelForCellType(root_ci->type, root_bel)) {
        return false;
    }

    IdString bel_type = ctx->getBelType(root_bel);
    // non-chain DSP
    if (root_ci->constr_children.size() == 1 && bel_type.in(id_PADD9, id_MULT9X9)) {
        return HimbaechelAPI::getClusterPlacement(cluster, root_bel, placement);
    }

    placement.clear();
    Loc root_loc = ctx->getBelLocation(root_bel);
    placement.emplace_back(root_ci, root_bel);

    Loc mult_loc = root_loc;
    for (auto child : root_ci->constr_children) {
        Loc child_loc;
        child_loc.y = root_loc.y;
        if (child->type == id_DUMMY_CELL) {
            child_loc.x = mult_loc.x + child->constr_x;
            child_loc.z = mult_loc.z + child->constr_z;
        } else {
            child_loc = gwu.get_dsp_next_in_chain(mult_loc, child->type);
            mult_loc = child_loc;
        }

        BelId child_bel = ctx->getBelByLocation(child_loc);
        if (child_bel == BelId() || !isValidBelForCellType(child->type, child_bel))
            return false;
        placement.emplace_back(child, child_bel);
    }
    return true;
}

void GowinImpl::notifyBelChange(BelId bel, CellInfo *cell)
{

    IdString bel_type = ctx->getBelType(bel);
    switch (bel_type.hash()) {
    case ID_LUT4: /* fall-through */
    case ID_DFF:
    case ID_ALU:
    case ID_RAM16SDP4:
    case ID_MUX2_LUT5:
    case ID_MUX2_LUT6:
    case ID_MUX2_LUT7:
    case ID_MUX2_LUT8:
        auto loc = ctx->getBelLocation(bel);
        if (fast_logic_cell.width() != ctx->getGridDimX() || fast_logic_cell.height() != ctx->getGridDimY())
            prepare_fast_logic_cell();
        fast_logic_cell.at(loc.x, loc.y).at(loc.z) = cell;
        return;
    }

    if (cell != nullptr && !is_dsp(cell)) {
        return;
    }
    if (cell == nullptr && dsp_bel2cell.count(bel) == 0) {
        return;
    }

    // trace DSP control networks
    IdString cell_type = id_DUMMY_CELL;
    if (cell != nullptr) {
        cell_type = cell->type;
    }
    Loc loc = ctx->getBelLocation(bel);
    Loc l = loc;
    l.z = gwu.get_dsp(loc.z);
    BelId dsp = ctx->getBelByLocation(l);
    l.z = gwu.get_dsp_macro(loc.z);
    BelId dsp_macro = ctx->getBelByLocation(l);

    if (cell) {
        bool mode9 = cell_type.in(id_PADD9, id_MULT9X9);
        if (mode9) {
            dsp_info[dsp_macro].mode9bit++;
        } else {
            dsp_info[dsp_macro].mode18bit++;
        }

        const auto &dsp_cell_data = fast_cell_info.at(cell->flat_index);
        if (dsp_cell_data.dsp_reset != nullptr) {
            dsp_info[dsp].reset[dsp_cell_data.dsp_reset->name]++;
        }
        if (dsp_cell_data.dsp_ce != nullptr) {
            dsp_info.at(dsp_macro).ce[dsp_cell_data.dsp_ce->name]++;
        }
        if (dsp_cell_data.dsp_clk != nullptr) {
            dsp_info.at(dsp_macro).clk[dsp_cell_data.dsp_clk->name]++;
        }
        dsp_bel2cell[bel] = cell;
    } else {
        bool mode9 = dsp_bel2cell.at(bel)->type.in(id_PADD9, id_MULT9X9);
        if (mode9) {
            dsp_info.at(dsp_macro).mode9bit--;
        } else {
            dsp_info.at(dsp_macro).mode18bit--;
        }

        const auto &dsp_cell_data = fast_cell_info.at(dsp_bel2cell.at(bel)->flat_index);
        if (dsp_cell_data.dsp_reset != nullptr) {
            dsp_info.at(dsp).reset.at(dsp_cell_data.dsp_reset->name)--;
        }
        if (dsp_cell_data.dsp_ce != nullptr) {
            dsp_info.at(dsp_macro).ce.at(dsp_cell_data.dsp_ce->name)--;
        }
        if (dsp_cell_data.dsp_clk != nullptr) {
            dsp_info.at(dsp_macro).clk.at(dsp_cell_data.dsp_clk->name)--;
        }
        dsp_bel2cell.erase(bel);
    }
}

void GowinImpl::configurePlacerHeap(PlacerHeapCfg &cfg)
{
    // Use cell groups to enforce a legalisation order
    cfg.cellGroups.emplace_back();
    cfg.cellGroups.back().insert(id_RAM16SDP4);
    cfg.cellGroups.emplace_back();
    cfg.cellGroups.back().insert(id_ALU);

    cfg.placeAllAtOnce = true;

    // Treat control and constants like IO buffers, because they have only one possible location
    cfg.ioBufTypes.insert(id_GOWIN_VCC);
    cfg.ioBufTypes.insert(id_GOWIN_GND);
    cfg.ioBufTypes.insert(id_PINCFG);
    cfg.ioBufTypes.insert(id_GSR);

    // Configure control-set-aware FF placement.  Without this HeAP places DFFs
    // without regard to which slices share CLK/CE/LSR control nets, leading
    // to massive thrash during legalisation on tight designs (88%% LUT4, 49%%
    // DFF utilisation triggers control-set congestion at certain tiles).
    //
    // GW5A slice control set groups (chipdb FF z-coords):
    //   Group 0: slices 0,1   (FF z=1, z=3)
    //   Group 1: slices 2,3   (FF z=5, z=7)
    //   Group 2: slices 4-7   (FF z=9, z=11, z=13, z=15) when has_DFF67
    //           else slices 4,5 only (FF z=9, z=11)
    cfg.ff_bel_bucket = id_DFF;
    cfg.ff_control_set_groups.clear();
    cfg.ff_control_set_groups.push_back({1, 3});
    cfg.ff_control_set_groups.push_back({5, 7});
    if (gwu.has_DFF67()) {
        cfg.ff_control_set_groups.push_back({9, 11, 13, 15});
    } else {
        cfg.ff_control_set_groups.push_back({9, 11});
    }
    // Iter-decay schedule for control-set window radius.  Earlier iters search
    // wider for matching control sets; later iters narrow as legalisation
    // converges.  Same shape as the upstream xilinx defaults.
    cfg.ctrl_set_max_radius = std::vector<int>{18, 15, 12, 9, 6, 3};

    cfg.get_cell_control_set = [](Context *ctx, const CellInfo *ci) {
        if (!is_dff(ci))
            return -1;
        // Build a unique int key from (CLK, CE, LSR, type-flavour).
        // Two DFFs with identical control nets and compatible types get
        // the same key; HeAP then groups them into the same control-set
        // bucket of a tile during legalisation.
        const NetInfo *clk = ci->getPort(id_CLK);
        const NetInfo *ce  = ci->getPort(id_CE);
        const NetInfo *lsr = nullptr;
        for (IdString p : {id_SET, id_RESET, id_PRESET, id_CLEAR}) {
            lsr = ci->getPort(p);
            if (lsr) break;
        }
        // Hash the three nets + cell type.  We use net pointer identities;
        // two FFs sharing the same NetInfo* on each port hash identically.
        // Type matters because incompatible_ffs() rules also apply.
        int32_t h = 17;
        h = h * 1000003 + (clk ? int32_t(clk->udata) : 0);
        h = h * 1000003 + (ce  ? int32_t(ce->udata)  : 0);
        h = h * 1000003 + (lsr ? int32_t(lsr->udata) : 0);
        // Mix in type to keep e.g. DFF and DFFE in different buckets even
        // when they share clock/ce nets (they cannot share a slice pair).
        h = h * 31 + int32_t(ci->type.index);
        // Avoid -1 (which means "no control set"); fold to non-negative.
        h &= 0x7fffffff;
        if (h == 0) h = 1;
        return h;
    };
}

void GowinImpl::drawBel(std::vector<GraphicElement> &g, GraphicElement::style_t style, IdString bel_type, Loc loc)
{
    GraphicElement el;
    el.type = GraphicElement::TYPE_BOX;
    el.style = style;
    switch (bel_type.index) {
    case id_LUT4.index:
        el.x1 = loc.x + 0.75;
        el.x2 = el.x1 + 0.08;
        el.y1 = loc.y + 0.1 + 0.1 * (loc.z / 2);
        el.y2 = el.y1 + 0.04;
        g.push_back(el);
        break;
    case id_ALU.index:
        el.x1 = loc.x + 0.75;
        el.x2 = el.x1 + 0.08;
        el.y1 = loc.y + 0.1 + 0.1 * (loc.z - BelZ::ALU0_Z);
        el.y2 = el.y1 + 0.04;
        g.push_back(el);
        break;
    case id_DFF.index:
        el.x1 = loc.x + 0.9;
        el.x2 = el.x1 + 0.02;
        el.y1 = loc.y + 0.1 + 0.1 * (loc.z / 2);
        el.y2 = el.y1 + 0.04;
        g.push_back(el);
        break;
    case id_RAM16SDP4.index:
        el.x1 = loc.x + 0.85;
        el.x2 = el.x1 + 0.01;
        el.y1 = loc.y + 0.1;
        el.y2 = el.y1 + 0.65;
        g.push_back(el);
        break;
    }
}

delay_t GowinImpl::estimateDelay(WireId src, WireId dst) const
{
    int sx, sy, dx, dy;
    tile_xy(ctx->chip_info, src.tile, sx, sy);
    tile_xy(ctx->chip_info, dst.tile, dx, dy);
    int dist_x = std::abs(dx - sx), dist_y = std::abs(dy - sy);
    return delay_c + delay_m * (std::max(dist_x - 4, 0) + std::max(dist_y - 4, 0) +
                                2 * (std::min(dist_x, 4) + std::min(dist_y, 4)));
}

} // namespace

NEXTPNR_NAMESPACE_END
