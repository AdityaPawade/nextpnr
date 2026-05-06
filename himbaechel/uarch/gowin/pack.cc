#include <map>

#include "design_utils.h"
#include "log.h"
#include "nextpnr.h"

#define HIMBAECHEL_CONSTIDS "uarch/gowin/constids.inc"
#include "himbaechel_constids.h"
#include "himbaechel_helpers.h"

#include "gowin.h"
#include "gowin_utils.h"
#include "pack.h"

#include <cinttypes>

NEXTPNR_NAMESPACE_BEGIN

// ===================================
// Global set/reset
// ===================================
void GowinPacker::pack_gsr(void)
{
    log_info("Pack GSR...\n");

    bool user_gsr = false;
    for (auto &cell : ctx->cells) {
        auto &ci = *cell.second;

        if (ci.type == id_GSR) {
            user_gsr = true;
            break;
        }
    }
    if (!user_gsr) {
        // make default GSR
        auto gsr_cell = std::make_unique<CellInfo>(ctx, id_GSR, id_GSR);
        gsr_cell->addInput(id_GSRI);
        gsr_cell->connectPort(id_GSRI, ctx->nets.at(ctx->id("$PACKER_VCC")).get());
        // Phase 7d fix: GSR.GSRI port maps to LSR0 wire of its slice (per
        // apicula chipdb extra_func gsr.wire="LSR0"). FFs placed in slots
        // 0,1 of the SAME slice would have R/S/C/P also targeting LSR0 ->
        // router LSR0 conflict ($PACKER_VCC vs $PACKER_GND).
        // Find the GSR bel location and reserve FF slots 0,1 there with
        // BLOCKER_FF cells (similar to RAM16SDP4 blocker pattern).
        ctx->cells[gsr_cell->name] = std::move(gsr_cell);
    }
    if (ctx->verbose) {
        if (user_gsr) {
            log_info("Have user GSR\n");
        } else {
            log_info("No user GSR. Make one.\n");
        }
    }
}

// ===================================
// Pin function configuration via wires
// ===================================
void GowinPacker::pack_pincfg(void)
{
    if (!gwu.has_PINCFG()) {
        return;
    }
    log_info("Pack PINCFG...\n");

    auto pincfg_cell = std::make_unique<CellInfo>(ctx, id_PINCFG, id_PINCFG);

    const int pin_cnt = gwu.has_I2CCFG() ? 5 : 4;
    for (int i = 0; i < pin_cnt; ++i) {
        IdString port = ctx->idf("UNK%d_VCC", i);
        pincfg_cell->addInput(port);
        if (i && gwu.need_CFGPINS_INVERSION()) {
            pincfg_cell->connectPort(port, ctx->nets.at(ctx->id("$PACKER_GND")).get());
        } else {
            pincfg_cell->connectPort(port, ctx->nets.at(ctx->id("$PACKER_VCC")).get());
        }
    }

    const ArchArgs &args = ctx->args;

    pincfg_cell->addInput(id_SSPI);
    if (args.options.count("sspi_as_gpio")) {
        pincfg_cell->connectPort(id_SSPI, ctx->nets.at(ctx->id("$PACKER_VCC")).get());
        pincfg_cell->setParam(id_SSPI, 1);
    } else {
        pincfg_cell->connectPort(id_SSPI, ctx->nets.at(ctx->id("$PACKER_GND")).get());
    }

    if (gwu.has_I2CCFG()) {
        pincfg_cell->addInput(id_I2C);
        if (args.options.count("i2c_as_gpio")) {
            pincfg_cell->connectPort(id_I2C, ctx->nets.at(ctx->id("$PACKER_VCC")).get());
            pincfg_cell->setParam(id_I2C, 1);
        } else {
            pincfg_cell->connectPort(id_I2C, ctx->nets.at(ctx->id("$PACKER_GND")).get());
        }
    }
    ctx->cells[pincfg_cell->name] = std::move(pincfg_cell);
}

// ===================================
// Global power regulator
// ===================================
void GowinPacker::pack_bandgap(void)
{
    if (!gwu.has_BANDGAP()) {
        return;
    }
    log_info("Pack BANDGAP...\n");

    bool user_bandgap = false;
    for (auto &cell : ctx->cells) {
        auto &ci = *cell.second;

        if (ci.type == id_BANDGAP) {
            user_bandgap = true;
            break;
        }
    }
    if (!user_bandgap) {
        // make default BANDGAP
        auto bandgap_cell = std::make_unique<CellInfo>(ctx, id_BANDGAP, id_BANDGAP);
        bandgap_cell->addInput(id_BGEN);
        bandgap_cell->connectPort(id_BGEN, ctx->nets.at(ctx->id("$PACKER_VCC")).get());
        ctx->cells[bandgap_cell->name] = std::move(bandgap_cell);
    }
    if (ctx->verbose) {
        if (user_bandgap) {
            log_info("Have user BANDGAP\n");
        } else {
            log_info("No user BANDGAP. Make one.\n");
        }
    }
}

// ===================================
// Replace INV with LUT
// ===================================
void GowinPacker::pack_inv(void)
{
    log_info("Pack INV...\n");

    for (auto &cell : ctx->cells) {
        auto &ci = *cell.second;

        if (ci.type == id_INV) {
            ci.type = id_LUT4;
            ci.renamePort(id_O, id_F);
            ci.renamePort(id_I, id_I3); // use D - it's simple for INIT
            ci.params[id_INIT] = Property(0x00ff);
        }
    }
}

// ===================================
// PLL
// ===================================
void GowinPacker::pack_pll(void)
{
    log_info("Pack PLL...\n");

    pool<BelId> used_pll_bels;

    for (auto &cell : ctx->cells) {
        auto &ci = *cell.second;

        if (ci.type.in(id_rPLL, id_PLLVR, id_PLLA)) {
            gwu.remove_brackets(&ci);

            // If CLKIN is connected to a special pin, then it makes sense
            // to try to place the PLL so that it uses a direct connection
            // to this pin.
            if (ci.bel == BelId()) {
                NetInfo *ni = ci.getPort(id_CLKIN);
                if (ni && ni->driver.cell && ni->driver.cell->bel != BelId()) {
                    BelId pll_bel = gwu.get_pll_bel(ni->driver.cell->bel, id_CLKIN_T);
                    if (ctx->debug) {
                        log_info("PLL clkin driver:%s at %s, PLL bel:%s\n", ctx->nameOf(ni->driver.cell),
                                 ctx->getBelName(ni->driver.cell->bel).str(ctx).c_str(),
                                 pll_bel != BelId() ? ctx->getBelName(pll_bel).str(ctx).c_str() : "NULL");
                    }
                    if (pll_bel != BelId() && used_pll_bels.count(pll_bel) == 0) {
                        used_pll_bels.insert(pll_bel);
                        ctx->bindBel(pll_bel, &ci, PlaceStrength::STRENGTH_LOCKED);
                        ci.disconnectPort(id_CLKIN);
                        ci.setParam(id_INSEL, std::string("CLKIN0"));
                    }
                }
            }
        }
    }
}

// ===================================
// ADC
// ===================================
void GowinPacker::pack_adc(void)
{
    log_info("Pack ADC...\n");

    for (auto &cell : ctx->cells) {
        auto &ci = *cell.second;

        if (is_adc(&ci)) {
            gwu.remove_brackets(&ci);
        }
    }
}

// ===================================
// HCLK -- CLKDIV and CLKDIV2 for now
// ===================================
void GowinPacker::pack_hclk(void)
{
    log_info("Pack HCLK cells...\n");

    // In the GW5A series, the CLKDIV2 can simultaneously transmit a signal to
    // both IOLOGIC and CLKDIV.
    if (gwu.has_5A_HCLK()) {
        return;
    }
    for (auto &cell : ctx->cells) {
        auto ci = cell.second.get();
        if (ci->type != id_CLKDIV)
            continue;
        NetInfo *hclk_in = ci->getPort(id_HCLKIN);
        if (hclk_in) {
            CellInfo *this_driver = hclk_in->driver.cell;
            if (this_driver && this_driver->type == id_CLKDIV2) {
                NetInfo *out = this_driver->getPort(id_CLKOUT);
                if (out->users.entries() > 1) {
                    // We could do as the IDE does sometimes and replicate the CLKDIV2 cell
                    // as many times as we need. For now, we keep things simple
                    log_error("CLKDIV2 that drives CLKDIV should drive no other cells\n");
                }
                ci->cluster = ci->name;
                this_driver->cluster = ci->name;
                ci->constr_children.push_back(this_driver);
                this_driver->constr_x = 0;
                this_driver->constr_y = 0;
                this_driver->constr_z = BelZ::CLKDIV2_0_Z - BelZ::CLKDIV_0_Z;
                this_driver->constr_abs_z = false;
            }
        }
    }
}

// ===================================
// DLLDLY
// ===================================
void GowinPacker::pack_dlldly(void)
{
    log_info("Pack DLLDLYs...\n");

    for (auto &cell : ctx->cells) {
        auto ci = cell.second.get();
        if (ci->type != id_DLLDLY)
            continue;
        NetInfo *clkin_net = ci->getPort(id_CLKIN);
        NetInfo *clkout_net = ci->getPort(id_CLKOUT);
        if (clkin_net == nullptr || clkout_net == nullptr) {
            log_error("%s cell has unconnected CLKIN or CLKOUT pins.\n", ctx->nameOf(ci));
        }
        CellInfo *clk_src = clkin_net->driver.cell;
        if (!is_io(clk_src)) {
            log_error("Clock source for DLLDLY %s is not IO: %s.\n", ctx->nameOf(ci), ctx->nameOf(clk_src));
        }
        // DLLDLY placement is fixed
        BelId io_bel = clk_src->bel;
        BelId dlldly_bel = gwu.get_dlldly_bel(io_bel);
        if (dlldly_bel == BelId()) {
            log_error("Can't use IO %s as input for DLLDLY %s.\n", ctx->nameOf(clk_src), ctx->nameOf(ci));
        }
        if (ctx->verbose) {
            log_info("  pack %s to use clock pin at %s\n", ctx->nameOf(ci), ctx->nameOfBel(io_bel));
        }
        ctx->bindBel(dlldly_bel, ci, STRENGTH_LOCKED);
        gwu.remove_brackets(ci);
    }
}

// =========================================
// Create entry points to the clock system
// =========================================
void GowinPacker::pack_buffered_nets(void)
{
    log_info("Pack buffered nets...\n");

    for (auto &net : ctx->nets) {
        NetInfo *ni = net.second.get();
        if (ni->driver.cell == nullptr || ni->users.empty() || net.first == ctx->id("$PACKER_GND") ||
            net.first == ctx->id("$PACKER_VCC")) {
            continue;
        }
        if (ni->attrs.count(id_CLOCK) == 0) {
            if (ctx->settings.count(id_NO_GP_CLOCK_ROUTING)) {
                continue;
            }
            // Count clock users (port = CLK*) — also used to decide whether
            // a GCLK-pin driver needs explicit BUFG (Phase 6 fix per Codex
            // round 12: high-fanout clocks need BUFG even on GCLK pins
            // because the implicit GP global routing fails for >50 loads).
            int clock_fanout = 0;
            for (auto usr : ni->users) {
                if (usr.port.in(id_CLKIN, id_CLK, id_CLK0, id_CLK1, id_CLK2, id_CLK3, id_CLKFB)) {
                    if (usr.port == id_CLK && usr.cell->attrs.count(id_LATCH))
                        continue;
                    ++clock_fanout;
                }
            }
            const int CLOCK_BUFG_FANOUT_THRESHOLD = 1;  // Phase 8: re-enable BUFG forcing for any clock-port user (disabled by 7f, but tribuf flow needs it)
            if (gwu.driver_is_clksrc(ni->driver) || (!gwu.driver_is_io(ni->driver))) {
                // Pre-Phase-6: skip if driver is already a clock source.
                // Phase 6: still skip UNLESS clock fanout exceeds threshold.
                // Top-level GCLK-pin clock with thousands of FF loads cannot
                // be auto-routed onto the global network without an explicit
                // BUFG — fall through to BUFG insertion below.
                if (clock_fanout < CLOCK_BUFG_FANOUT_THRESHOLD) {
                    continue;
                }
                // High-fanout clock on GCLK pin: force BUFG insertion.
                if (ctx->verbose) {
                    log_info("Force BUFG on high-fanout (%d) GCLK-pin net '%s'\n",
                             clock_fanout, ctx->nameOf(ni));
                }
            } else {
                // Driver is an IBUF on a non-GCLK pin: existing behavior
                // (BUFG only if clock-port users exist).
                if (clock_fanout == 0) {
                    continue;
                }
                if (ctx->verbose) {
                    log_info("Add buffering to a potentially clock network '%s'\n", ctx->nameOf(ni));
                }
            }
        }

        // make new BUF cell single user for the net driver
        IdString buf_name = ctx->idf("%s_BUFG", net.first.c_str(ctx));
        ctx->createCell(buf_name, id_BUFG);
        CellInfo *buf_ci = ctx->cells.at(buf_name).get();
        buf_ci->addInput(id_I);
        // move driver
        CellInfo *driver_cell = ni->driver.cell;
        IdString driver_port = ni->driver.port;

        driver_cell->movePortTo(driver_port, buf_ci, id_O);
        buf_ci->connectPorts(id_I, driver_cell, driver_port);
    }
}

// =========================================
// Create DQCEs
// =========================================
void GowinPacker::pack_dqce(void)
{
    log_info("Pack DQCE cells...\n");

    // At the placement stage, nothing can be said definitively about DQCE,
    // so we make user cells virtual but allocate all available bels by
    // creating and placing cells - we will use some of them after, and
    // delete the rest.
    // We do this here because the decision about which physical DQCEs to
    // use is made during routing, but some of the information (let’s say
    // mapping cell pins -> bel pins) is filled in before routing.
    bool grab_bels = false;
    for (auto &cell : ctx->cells) {
        auto &ci = *cell.second;
        if (ci.type == id_DQCE) {
            ci.pseudo_cell = std::make_unique<RegionPlug>(Loc(0, 0, 0));
            grab_bels = true;
        }
    }
    if (grab_bels) {
        for (int i = 0; i < 32; ++i) {
            BelId dqce_bel = gwu.get_dqce_bel(ctx->idf("SPINE%d", i));
            if (dqce_bel != BelId()) {
                IdString dqce_name = ctx->idf("$PACKER_DQCE_SPINE%d", i);
                CellInfo *dqce = ctx->createCell(dqce_name, id_DQCE);
                dqce->addInput(id_CE);
                ctx->bindBel(dqce_bel, dqce, STRENGTH_LOCKED);
            }
        }
    }
}

// =========================================
// Create DCSs
// =========================================
void GowinPacker::pack_dcs(void)
{
    log_info("Pack DCS cells...\n");

    // At the placement stage, nothing can be said definitively about DCS,
    // so we make user cells virtual but allocate all available bels by
    // creating and placing cells - we will use some of them after, and
    // delete the rest.
    // We do this here because the decision about which physical DCEs to
    // use is made during routing, but some of the information (let’s say
    // mapping cell pins -> bel pins) is filled in before routing.
    bool grab_bels = false;
    for (auto &cell : ctx->cells) {
        auto &ci = *cell.second;
        if (ci.type == id_DCS) {
            ci.pseudo_cell = std::make_unique<RegionPlug>(Loc(0, 0, 0));
            grab_bels = true;
        }
    }
    if (grab_bels) {
        for (int i = 0; i < 8; ++i) {
            BelId dcs_bel = gwu.get_dcs_bel(ctx->idf("P%d%dA", 1 + (i % 4), 6 + (i >> 2)));
            if (dcs_bel != BelId()) {
                IdString dcs_name = ctx->idf("$PACKER_DCS_SPINE%d", 8 * (i % 4) + 6 + (i >> 2));
                CellInfo *dcs = ctx->createCell(dcs_name, id_DCS);
                ctx->copyBelPorts(dcs_name, dcs_bel);
                ctx->bindBel(dcs_bel, dcs, STRENGTH_LOCKED);
            }
        }
    }
}

// =========================================
// Create DHCENs
// =========================================
void GowinPacker::pack_dhcens(void)
{
    // Allocate all available dhcen bels; we will find out which of them
    // will actually be used during the routing process.
    bool grab_bels = false;
    for (auto &cell : ctx->cells) {
        auto &ci = *cell.second;
        if (ci.type == id_DHCEN) {
            ci.pseudo_cell = std::make_unique<RegionPlug>(Loc(0, 0, 0));
            grab_bels = true;
        }
    }
    if (grab_bels) {
        // sane message if new primitives are used with old bases
        auto buckets = ctx->getBelBuckets();
        NPNR_ASSERT_MSG(std::find(buckets.begin(), buckets.end(), id_DHCEN) != buckets.end(),
                        "There are no DHCEN bels to use.");
        int i = 0;
        for (auto &bel : ctx->getBelsInBucket(ctx->getBelBucketForCellType(id_DHCEN))) {
            IdString dhcen_name = ctx->idf("$PACKER_DHCEN_%d", ++i);
            CellInfo *dhcen = ctx->createCell(dhcen_name, id_DHCEN);
            dhcen->addInput(id_CE);
            ctx->bindBel(bel, dhcen, STRENGTH_LOCKED);
        }
    }
}

// =========================================
// Enable UserFlash
// =========================================
void GowinPacker::pack_userflash(bool have_emcu)
{
    log_info("Pack UserFlash cells...\n");
    std::vector<std::unique_ptr<CellInfo>> new_cells;

    for (auto &cell : ctx->cells) {
        auto &ci = *cell.second;
        if (!is_userflash(&ci)) {
            continue;
        }

        if (ci.type.in(id_FLASH96K, id_FLASH256K, id_FLASH608K)) {
            // enable
            ci.addInput(id_INUSEN);
            ci.connectPort(id_INUSEN, ctx->nets.at(ctx->id("$PACKER_GND")).get());
        }
        gwu.remove_brackets(&ci);

        if (have_emcu) {
            continue;
        }

        // add invertor
        int lut_idx = 0;
        auto add_inv = [&](IdString port, PortType port_type) {
            if (!gwu.port_used(&ci, port)) {
                return;
            }

            std::unique_ptr<CellInfo> lut_cell =
                    gwu.create_cell(gwu.create_aux_name(ci.name, lut_idx, "_lut$"), id_LUT4);
            new_cells.push_back(std::move(lut_cell));
            CellInfo *lut = new_cells.back().get();
            lut->addInput(id_I0);
            lut->addOutput(id_F);
            lut->setParam(id_INIT, 0x5555);
            ++lut_idx;

            if (port_type == PORT_IN) {
                ci.movePortTo(port, lut, id_I0);
                lut->connectPorts(id_F, &ci, port);
            } else {
                ci.movePortTo(port, lut, id_F);
                ci.connectPorts(port, lut, id_I0);
            }
        };
        for (auto pin : ci.ports) {
            if (pin.second.type == PORT_OUT) {
                add_inv(pin.first, PORT_OUT);
            } else {
                if (pin.first == id_INUSEN) {
                    continue;
                }
                if (ci.type == id_FLASH608K && pin.first.in(id_XADR0, id_XADR1, id_XADR2, id_XADR3, id_XADR4, id_XADR5,
                                                            id_XADR6, id_XADR7, id_XADR8)) {
                    continue;
                }
                add_inv(pin.first, PORT_IN);
            }
        }
    }
    for (auto &ncell : new_cells) {
        ctx->cells[ncell->name] = std::move(ncell);
    }
}

// =========================================
// Create EMCU
// =========================================
void GowinPacker::pack_emcu_and_flash(void)
{
    log_info("Pack EMCU and UserFlash cells...\n");
    std::vector<std::unique_ptr<CellInfo>> new_cells;

    bool have_emcu = false;
    for (auto &cell : ctx->cells) {
        auto &ci = *cell.second;
        if (!is_emcu(&ci)) {
            continue;
        }
        have_emcu = true;

        gwu.remove_brackets(&ci);

        // The flash data bus is connected directly to the CPU so just disconnect these networks
        // also other non-switched networks
        ci.disconnectPort(ctx->id("DAPNTDOEN"));
        ci.disconnectPort(ctx->id("DAPNTRST"));
        ci.disconnectPort(ctx->id("DAPTDO"));
        ci.disconnectPort(ctx->id("DAPTDI"));
        ci.disconnectPort(ctx->id("TARGFLASH0HREADYMUX"));
        ci.disconnectPort(ctx->id("TARGEXP0HAUSER"));
        ci.disconnectPort(ctx->id("TARGFLASH0EXRESP"));
        ci.disconnectPort(ctx->id("PORESETN"));
        ci.disconnectPort(ctx->id("SYSRESETN"));
        ci.disconnectPort(ctx->id("DAPSWDITMS"));
        ci.disconnectPort(ctx->id("DAPSWCLKTCK"));
        ci.disconnectPort(ctx->id("TPIUTRACECLK"));
        for (int i = 0; i < 32; ++i) {
            if (i < 4) {
                if (i < 3) {
                    ci.disconnectPort(ctx->idf("TARGFLASH0HSIZE%d", i));
                    ci.disconnectPort(ctx->idf("TARGFLASH0HBURST%d", i));
                    ci.disconnectPort(ctx->idf("TARGFLASH0HRUSER%d", i));
                    ci.disconnectPort(ctx->idf("INITEXP0HRUSER%d", i));
                }
                // ci.disconnectPort(ctx->idf("TARGFLASH0HPROT%d", i));
                ci.disconnectPort(ctx->idf("TARGEXP0HWUSER%d", i));
                ci.disconnectPort(ctx->idf("MTXREMAP%d", i));
            }
            // ins
            ci.disconnectPort(ctx->idf("TARGFLASH0HRDATA%d", i));
        }
    }
    pack_userflash(have_emcu);
}

void GowinPacker::run(void)
{
    handle_constants();
    pack_iobs();
    ctx->check();

    pack_i3c();
    ctx->check();

    pack_mipi();
    ctx->check();

    pack_diff_iobs();
    ctx->check();

    pack_io_regs();
    ctx->check();

    pack_iodelay();
    ctx->check();

    pack_iem();
    ctx->check();

    pack_iologic();
    ctx->check();

    pack_io16();
    ctx->check();

    pack_gsr();
    ctx->check();

    pack_pincfg();
    ctx->check();

    pack_hclk();
    ctx->check();

    pack_dlldly();
    ctx->check();

    pack_bandgap();
    ctx->check();

    pack_wideluts();
    ctx->check();

    pack_alus();
    ctx->check();

    pack_ssram();
    ctx->check();

    pack_latches();
    ctx->check();

    constrain_lutffs();
    ctx->check();

    pair_alu_dffs();
    ctx->check();

    // Insert passthrough LUT4 (INIT=0xff00) for any DFF still unpaired.
    // This mirrors what Gowin EDA does (verified via gowin_unpack of a
    // reference bitstream: 1325+ buffer LUTs with INIT=0xff00).
    // Round 17: gate buffer-LUT trick behind env var GOWIN_BUFFER_LUTS=1.
    // Default: OFF -> orphan FFs route via REG_SD path natively.
    if (getenv("GOWIN_BUFFER_LUTS") != nullptr) {
        log_info("GOWIN_BUFFER_LUTS=1: applying buffer-LUT trick.\n");
        insert_buffer_luts_for_orphan_dffs();
    } else {
        log_info("GOWIN_BUFFER_LUTS not set: SKIPPING buffer-LUT trick.\n");
    }
    ctx->check();

    // Phase 7 LSR canonicalize — MOVED to end of pack pipeline (Phase 7c)
    // so it catches VCC connections added by pack_pll, pack_bsram, pack_dsp,
    // pack_buffered_nets, pack_dqce, etc.

    // constrain_orphan_lutffs();  // disabled: violates slice_valid (FF.D must == LUT.F)
    // ctx->check();

    pack_pll();
    ctx->check();

    pack_adc();
    ctx->check();

    pack_bsram();
    ctx->check();

    pack_dsp();
    ctx->check();

    pack_inv();
    ctx->check();

    pack_buffered_nets();
    ctx->check();

    pack_emcu_and_flash();
    ctx->check();

    pack_dhcens();
    ctx->check();

    pack_dqce();
    ctx->check();

    pack_dcs();
    ctx->check();

    // Phase 7c: LSR canonicalize MOVED here from earlier in pipeline so it
    // catches VCC connections added by pack_pll, pack_bsram, pack_dsp,
    // pack_buffered_nets, pack_dqce, pack_dcs, etc.
    normalize_inactive_lsr_ports();
    ctx->check();

    ctx->fixupHierarchy();
    ctx->check();

    // R23: dump FF control sets to CSV (gated on env var GOWIN_FF_CSV_DUMP)
    dump_ff_control_sets();
}

void gowin_pack(Context *ctx)
{
    GowinPacker packer(ctx);
    packer.run();
}

NEXTPNR_NAMESPACE_END
