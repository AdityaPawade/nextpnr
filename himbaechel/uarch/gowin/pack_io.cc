#include "design_utils.h"
#include "log.h"
#include "nextpnr.h"

#define HIMBAECHEL_CONSTIDS "uarch/gowin/constids.inc"
#include "himbaechel_constids.h"
#include "himbaechel_helpers.h"

#include "gowin.h"
#include "gowin_utils.h"
#include "pack.h"

#include <algorithm>
#include <cinttypes>
#include <cstdlib>
#include <cstring>
#include <vector>

NEXTPNR_NAMESPACE_BEGIN

// ===================================
// IO
// ===================================
// create IOB connections for gowin_pack
// can be called repeatedly when switching inputs, disabled outputs do not change
void GowinPacker::make_iob_nets(CellInfo &iob)
{
    for (const auto &port : iob.ports) {
        const NetInfo *net = iob.getPort(port.first);
        std::string connected_net = "NET";
        if (net != nullptr) {
            if (ctx->verbose) {
                log_info("%s: %s - %s\n", ctx->nameOf(&iob), port.first.c_str(ctx), ctx->nameOf(net));
            }
            if (net->name == ctx->id("$PACKER_VCC")) {
                connected_net = "VCC";
            } else if (net->name == ctx->id("$PACKER_GND")) {
                connected_net = "GND";
            }
            iob.setParam(ctx->idf("NET_%s", port.first.c_str(ctx)), connected_net);
        }
    }
}

void GowinPacker::config_simple_io(CellInfo &ci)
{
    if (ci.type.in(id_TBUF, id_IOBUF)) {
        return;
    }
    log_info("simple:%s\n", ctx->nameOf(&ci));
    ci.addInput(id_OEN);
    if (ci.type == id_OBUF) {
        ci.connectPort(id_OEN, ctx->nets.at(ctx->id("$PACKER_GND")).get());
    } else {
        NPNR_ASSERT(ci.type == id_IBUF);
        ci.connectPort(id_OEN, ctx->nets.at(ctx->id("$PACKER_VCC")).get());
    }
}

void GowinPacker::config_bottom_row(CellInfo &ci, Loc loc, uint8_t cnd)
{
    if (!gwu.has_bottom_io_cnds()) {
        return;
    }
    if (!ci.type.in(id_OBUF, id_TBUF, id_IOBUF)) {
        return;
    }
    if (loc.z != BelZ::IOBA_Z) {
        return;
    }
    auto connect_io_wire = [&](IdString port, IdString net_name) {
        // XXX it is very convenient that nothing terrible happens in case
        // of absence/presence of a port
        ci.disconnectPort(port);
        ci.addInput(port);
        if (net_name == id_VSS) {
            ci.connectPort(port, ctx->nets.at(ctx->id("$PACKER_GND")).get());
        } else {
            NPNR_ASSERT(net_name == id_VCC);
            ci.connectPort(port, ctx->nets.at(ctx->id("$PACKER_VCC")).get());
        }
    };

    IdString wire_a_net = gwu.get_bottom_io_wire_a_net(cnd);
    connect_io_wire(id_BOTTOM_IO_PORT_A, wire_a_net);

    IdString wire_b_net = gwu.get_bottom_io_wire_b_net(cnd);
    connect_io_wire(id_BOTTOM_IO_PORT_B, wire_b_net);
}

// Attributes of deleted cells are copied
void GowinPacker::trim_nextpnr_iobs(void)
{
    // Trim nextpnr IOBs - assume IO buffer insertion has been done in synthesis
    const pool<CellTypePort> top_ports{
            CellTypePort(id_IBUF, id_I),
            CellTypePort(id_OBUF, id_O),
            CellTypePort(id_TBUF, id_O),
            CellTypePort(id_IOBUF, id_IO),
    };
    std::vector<IdString> to_remove;
    for (auto &cell : ctx->cells) {
        auto &ci = *cell.second;
        if (!ci.type.in(ctx->id("$nextpnr_ibuf"), ctx->id("$nextpnr_obuf"), ctx->id("$nextpnr_iobuf")))
            continue;
        NetInfo *i = ci.getPort(id_I);
        if (i && i->driver.cell) {
            if (!top_ports.count(CellTypePort(i->driver)))
                log_error("Top-level port '%s' driven by illegal port %s.%s\n", ctx->nameOf(&ci),
                          ctx->nameOf(i->driver.cell), ctx->nameOf(i->driver.port));
            for (const auto &attr : ci.attrs) {
                i->driver.cell->setAttr(attr.first, attr.second);
            }
        }
        NetInfo *o = ci.getPort(id_O);
        if (o) {
            for (auto &usr : o->users) {
                if (!top_ports.count(CellTypePort(usr)))
                    log_error("Top-level port '%s' driving illegal port %s.%s\n", ctx->nameOf(&ci),
                              ctx->nameOf(usr.cell), ctx->nameOf(usr.port));
                for (const auto &attr : ci.attrs) {
                    usr.cell->setAttr(attr.first, attr.second);
                }
                // network/port attributes that can be set in the
                // restriction file and that need to be transferred to real
                // networks before nextpnr buffers are removed.
                NetInfo *dst_net = usr.cell->getPort(id_O);
                if (dst_net != nullptr) {
                    for (const auto &attr : o->attrs) {
                        if (!attr.first.in(id_CLOCK)) {
                            continue;
                        }
                        dst_net->attrs[attr.first] = attr.second;
                    }
                }
            }
        }
        NetInfo *io = ci.getPort(id_IO);
        if (io && io->driver.cell) {
            if (!top_ports.count(CellTypePort(io->driver)))
                log_error("Top-level port '%s' driven by illegal port %s.%s\n", ctx->nameOf(&ci),
                          ctx->nameOf(io->driver.cell), ctx->nameOf(io->driver.port));
            for (const auto &attr : ci.attrs) {
                io->driver.cell->setAttr(attr.first, attr.second);
            }
        }
        ci.disconnectPort(id_I);
        ci.disconnectPort(id_O);
        ci.disconnectPort(id_IO);
        to_remove.push_back(ci.name);
    }
    for (IdString cell_name : to_remove)
        ctx->cells.erase(cell_name);
}

BelId GowinPacker::bind_io(CellInfo &ci)
{
    BelId bel = ctx->getBelByNameStr(ci.attrs.at(id_BEL).as_string());
    if (bel == BelId()) {
        log_error("No bel named %s\n", ci.attrs.at(id_BEL).as_string().c_str());
    }
    if (!ctx->checkBelAvail(bel)) {
        log_error("Can't place %s at %s because it's already taken by %s\n", ctx->nameOf(&ci), ctx->nameOfBel(bel),
                  ctx->nameOf(ctx->getBoundBelCell(bel)));
    }
    ci.unsetAttr(id_BEL);
    ctx->bindBel(bel, &ci, PlaceStrength::STRENGTH_LOCKED);
    return bel;
}

void GowinPacker::pack_iobs(void)
{
    log_info("Pack IOBs...\n");
    trim_nextpnr_iobs();
    std::vector<IdString> cells_to_remove;

    for (auto &cell : ctx->cells) {
        CellInfo &ci = *cell.second;
        if (!is_io(&ci)) {
            continue;
        }
        // Special case of OBUF without input - we delete such things.
        if (ci.type == id_OBUF && !ci.getPort(id_I)) {
            ci.disconnectPort(id_O);
            cells_to_remove.push_back(ci.name);
            continue;
        }

        if (ci.attrs.count(id_BEL) == 0) {
            log_error("Unconstrained IO:%s\n", ctx->nameOf(&ci));
        }
        BelId io_bel = bind_io(ci);
        Loc io_loc = ctx->getBelLocation(io_bel);
        if (io_loc.y == ctx->getGridDimY() - 1) {
            config_bottom_row(ci, io_loc);
        }
        if (gwu.is_simple_io_bel(io_bel)) {
            config_simple_io(ci);
        }
        make_iob_nets(ci);
    }

    for (auto cell : cells_to_remove) {
        ctx->cells.erase(cell);
    }
}

// ===================================
// Differential IO
// ===================================

std::pair<CellInfo *, CellInfo *> GowinPacker::get_pn_cells(const CellInfo &ci)
{
    CellInfo *p, *n;
    switch (ci.type.hash()) {
    case ID_ELVDS_TBUF: /* fall-through */
    case ID_TLVDS_TBUF: /* fall-through */
    case ID_ELVDS_OBUF: /* fall-through */
    case ID_TLVDS_OBUF:
        p = net_only_drives(ctx, ci.ports.at(id_O).net, is_iob, id_I, true);
        n = net_only_drives(ctx, ci.ports.at(id_OB).net, is_iob, id_I, true);
        break;
    case ID_TLVDS_IBUF_ADC: /* fall-through */
    case ID_ELVDS_IBUF:     /* fall-through */
    case ID_TLVDS_IBUF:
        p = net_driven_by(ctx, ci.ports.at(id_I).net, is_iob, id_O);
        n = net_driven_by(ctx, ci.ports.at(id_IB).net, is_iob, id_O);
        break;
    case ID_ELVDS_IOBUF: /* fall-through */
    case ID_TLVDS_IOBUF:
        p = net_only_drives(ctx, ci.ports.at(id_IO).net, is_iob, id_I);
        n = net_only_drives(ctx, ci.ports.at(id_IOB).net, is_iob, id_I);
        break;
    default:
        log_error("Bad diff IO '%s' type '%s'\n", ctx->nameOf(&ci), ci.type.c_str(ctx));
    }
    return std::make_pair(p, n);
}

void GowinPacker::mark_iobs_as_diff(CellInfo &ci, std::pair<CellInfo *, CellInfo *> &pn_cells)
{
    pn_cells.first->setParam(id_DIFF, std::string("P"));
    pn_cells.first->setParam(id_DIFF_TYPE, ci.type.str(ctx));
    pn_cells.second->setParam(id_DIFF, std::string("N"));
    pn_cells.second->setParam(id_DIFF_TYPE, ci.type.str(ctx));
    if (ci.params.count(id_ADC_IO)) {
        pn_cells.first->setParam(id_ADC_IO, ci.params.at(id_ADC_IO));
        pn_cells.second->setParam(id_ADC_IO, ci.params.at(id_ADC_IO));
    }
}

void GowinPacker::switch_diff_ports(CellInfo &ci, std::pair<CellInfo *, CellInfo *> &pn_cells,
                                    std::vector<IdString> &nets_to_remove)
{
    CellInfo *iob_p = pn_cells.first;
    CellInfo *iob_n = pn_cells.second;

    if (ci.type.in(id_TLVDS_TBUF, id_TLVDS_OBUF, id_ELVDS_TBUF, id_ELVDS_OBUF)) {
        nets_to_remove.push_back(ci.getPort(id_O)->name);
        ci.disconnectPort(id_O);
        nets_to_remove.push_back(ci.getPort(id_OB)->name);
        ci.disconnectPort(id_OB);
        nets_to_remove.push_back(iob_n->getPort(id_I)->name);
        iob_n->disconnectPort(id_I);

        if (ci.type.in(id_TLVDS_TBUF, id_ELVDS_TBUF)) {
            NetInfo *oen_net = iob_n->getPort(id_OEN);
            if (oen_net != nullptr) {
                nets_to_remove.push_back(oen_net->name);
            }
            iob_n->disconnectPort(id_OEN);
            iob_p->disconnectPort(id_OEN);
            ci.movePortTo(id_OEN, iob_p, id_OEN);

            // MIPI
            if (ci.params.count(id_MIPI_OBUF)) {
                iob_p->setParam(id_MIPI_OBUF, 1);
                iob_n->setParam(id_MIPI_OBUF, 1);
                ci.movePortTo(id_IB, iob_n, id_I);
                iob_p->copyPortTo(id_OEN, iob_n, id_OEN);
            }
        }
        iob_p->disconnectPort(id_I);
        ci.movePortTo(id_I, iob_p, id_I);
        return;
    }
    if (ci.type.in(id_TLVDS_IBUF, id_ELVDS_IBUF)) {
        nets_to_remove.push_back(ci.getPort(id_I)->name);
        ci.disconnectPort(id_I);
        nets_to_remove.push_back(ci.getPort(id_IB)->name);
        ci.disconnectPort(id_IB);
        iob_n->disconnectPort(id_O);
        iob_p->disconnectPort(id_O);
        ci.movePortTo(id_O, iob_p, id_O);
        return;
    }
    if (ci.type.in(id_TLVDS_IOBUF, id_ELVDS_IOBUF)) {
        nets_to_remove.push_back(ci.getPort(id_IO)->name);
        ci.disconnectPort(id_IO);
        nets_to_remove.push_back(ci.getPort(id_IOB)->name);
        ci.disconnectPort(id_IOB);
        nets_to_remove.push_back(iob_n->getPort(id_I)->name);
        iob_n->disconnectPort(id_I);
        iob_n->disconnectPort(id_OEN);

        iob_p->disconnectPort(id_OEN);
        ci.movePortTo(id_OEN, iob_p, id_OEN);
        iob_p->disconnectPort(id_I);
        ci.movePortTo(id_I, iob_p, id_I);
        iob_p->disconnectPort(id_O);
        ci.movePortTo(id_O, iob_p, id_O);
        return;
    }
    if (ci.type.in(id_TLVDS_IBUF_ADC)) {
        nets_to_remove.push_back(ci.getPort(id_I)->name);
        ci.disconnectPort(id_I);
        nets_to_remove.push_back(ci.getPort(id_IB)->name);
        ci.disconnectPort(id_IB);
        iob_p->disconnectPort(id_O);
        iob_n->disconnectPort(id_O);

        ci.movePortTo(id_ADCEN, iob_p, id_ADCEN);
        return;
    }
}

// ===================================
// I3C
// ===================================
void GowinPacker::pack_i3c(void)
{
    log_info("Pack I3C IOs...\n");
    std::vector<IdString> cells_to_remove;

    for (auto &cell : ctx->cells) {
        CellInfo &ci = *cell.second;
        if (!is_i3c(&ci)) {
            continue;
        }
        // check for I3C-capable pin A
        CellInfo *iob = net_only_drives(ctx, ci.ports.at(id_IO).net, is_iob, id_I);
        if (iob == nullptr || iob->bel == BelId()) {
            log_error("I3C %s IO is not connected to the input pin or the pin is not constrained.\n", ctx->nameOf(&ci));
        }
        BelId iob_bel = iob->bel;
        Loc iob_loc = ctx->getBelLocation(iob_bel);

        if (!gwu.get_i3c_capable(iob_loc.x, iob_loc.y)) {
            log_error("Can't place %s. Not I3C capable X%dY%d.\n", ctx->nameOf(&ci), iob_loc.x, iob_loc.y);
        }
        ci.disconnectPort(id_IO);
        iob->disconnectPort(id_I);
        ci.movePortTo(id_I, iob, id_I);
        ci.movePortTo(id_O, iob, id_O);
        iob->disconnectPort(id_OEN);
        ci.movePortTo(id_MODESEL, iob, id_OEN);

        iob->setParam(id_I3C_IOBUF, 1);
        cells_to_remove.push_back(ci.name);
    }

    for (auto cell : cells_to_remove) {
        ctx->cells.erase(cell);
    }
}

// ===================================
// MIPI IO
// ===================================
void GowinPacker::pack_mipi(void)
{
    log_info("Pack MIPI IOs...\n");
    std::vector<std::unique_ptr<CellInfo>> new_cells;

    for (auto &cell : ctx->cells) {
        CellInfo &ci = *cell.second;
        if (!is_mipi(&ci)) {
            continue;
        }
        switch (ci.type.hash()) {
        case ID_MIPI_OBUF_A: /* fall-through */
        case ID_MIPI_OBUF: {
            // check for MIPI-capable pin
            CellInfo *out_iob = net_only_drives(ctx, ci.ports.at(id_O).net, is_iob, id_I, true);
            if (out_iob == nullptr || out_iob->bel == BelId()) {
                log_error("MIPI %s is not connected to the output pin or the pin is not constrained.\n",
                          ctx->nameOf(&ci));
            }
            if (out_iob->params.count(id_I3C_IOBUF)) {
                log_error("Can't place MIPI %s. Conflict with I3C %s.\n", ctx->nameOf(&ci), ctx->nameOf(out_iob));
            }
            BelId iob_bel = out_iob->bel;
            Loc iob_loc = ctx->getBelLocation(iob_bel);
            iob_loc.z = BelZ::MIPIOBUF_Z;
            BelId mipi_bel = ctx->getBelByLocation(iob_loc);
            if (mipi_bel == BelId()) {
                log_error("Can't place MIPI %s at X%dY%d/IOBA.\n", ctx->nameOf(&ci), iob_loc.x, iob_loc.y);
            }

            if (ci.type == id_MIPI_OBUF_A) {
                // if serialization is used then IL and input of serializator must be in the same network
                NetInfo *i_net = ci.getPort(id_I);
                NetInfo *il_net = ci.getPort(id_IL);
                if (i_net != il_net) {
                    if (i_net != nullptr && is_iologico(i_net->driver.cell)) {
                        if (i_net->driver.cell->getPort(id_D0) != ci.getPort(id_IL)) {
                            log_error("MIPI %s port IL and IOLOGIC %s port D0 are in differrent networks!\n",
                                      ctx->nameOf(&ci), ctx->nameOf(i_net->driver.cell));
                        }
                    } else {
                        log_error("MIPI %s ports IL and I are in differrent networks!\n", ctx->nameOf(&ci));
                    }
                }
                ci.disconnectPort(id_IL);
            }

            ctx->bindBel(mipi_bel, &ci, PlaceStrength::STRENGTH_LOCKED);

            // Create TBUF with additional input IB
            IdString mipi_tbuf_name = gwu.create_aux_name(ci.name);
            new_cells.push_back(gwu.create_cell(mipi_tbuf_name, id_TLVDS_TBUF));

            CellInfo *mipi_tbuf = new_cells.back().get();
            mipi_tbuf->addInput(id_I);
            mipi_tbuf->addInput(id_IB);
            mipi_tbuf->addOutput(id_O);
            mipi_tbuf->addOutput(id_OB);
            mipi_tbuf->addInput(id_OEN);
            ci.movePortTo(id_I, mipi_tbuf, id_I);
            ci.movePortTo(id_IB, mipi_tbuf, id_IB);
            ci.movePortTo(id_O, mipi_tbuf, id_O);
            ci.movePortTo(id_OB, mipi_tbuf, id_OB);
            ci.movePortTo(id_MODESEL, mipi_tbuf, id_OEN);

            mipi_tbuf->setParam(id_MIPI_OBUF, 1);
        } break;
        case ID_MIPI_IBUF: {
            // check for MIPI-capable pin A
            CellInfo *in_iob = net_only_drives(ctx, ci.ports.at(id_IO).net, is_iob, id_I);
            if (in_iob == nullptr || in_iob->bel == BelId()) {
                log_error("MIPI %s IO is not connected to the input pin or the pin is not constrained.\n",
                          ctx->nameOf(&ci));
            }
            // check A IO placing
            if (in_iob->params.count(id_I3C_IOBUF)) {
                log_error("Can't place MIPI %s. Conflict with I3C %s.\n", ctx->nameOf(&ci), ctx->nameOf(in_iob));
            }
            BelId iob_bel = in_iob->bel;
            Loc iob_loc = ctx->getBelLocation(iob_bel);
            if (iob_loc.z != BelZ::IOBA_Z) {
                log_error("MIPI %s IO pin must be connected to the A IO pin.\n", ctx->nameOf(&ci));
            }

            iob_loc.z = BelZ::MIPIIBUF_Z;
            BelId mipi_bel = ctx->getBelByLocation(iob_loc);
            if (mipi_bel == BelId()) {
                log_error("Can't place MIPI %s at X%dY%d/IOBA.\n", ctx->nameOf(&ci), iob_loc.x, iob_loc.y);
            }

            // check for MIPI-capable pin B
            CellInfo *inb_iob = net_only_drives(ctx, ci.ports.at(id_IOB).net, is_iob, id_I);
            if (inb_iob == nullptr || inb_iob->bel == BelId()) {
                log_error("MIPI %s IOB is not connected to the input pin or the pin is not constrained.\n",
                          ctx->nameOf(&ci));
            }
            // check B IO placing
            if (inb_iob->params.count(id_I3C_IOBUF)) {
                log_error("Can't place MIPI %s. Conflict with I3C %s.\n", ctx->nameOf(&ci), ctx->nameOf(inb_iob));
            }
            BelId iobb_bel = inb_iob->bel;
            Loc iobb_loc = ctx->getBelLocation(iobb_bel);
            if (iobb_loc.z != BelZ::IOBB_Z || iobb_loc.x != iob_loc.x || iobb_loc.y != iob_loc.y) {
                log_error("MIPI %s IOB pin must be connected to the B IO pin.\n", ctx->nameOf(&ci));
            }
            // MIPI IBUF uses next pair of IOs too
            Loc iob_next_loc(iob_loc);
            ++iob_next_loc.x;
            iob_next_loc.z = BelZ::IOBA_Z;
            CellInfo *inc_iob = ctx->getBoundBelCell(ctx->getBelByLocation(iob_next_loc));
            iob_next_loc.z = BelZ::IOBB_Z;
            CellInfo *other_cell_b = ctx->getBoundBelCell(ctx->getBelByLocation(iob_next_loc));
            if (inc_iob != nullptr || other_cell_b != nullptr) {
                log_error("MIPI %s cannot be placed in same IO with %s.\n", ctx->nameOf(&ci),
                          inc_iob == nullptr ? ctx->nameOf(other_cell_b) : ctx->nameOf(inc_iob));
            }

            ctx->bindBel(mipi_bel, &ci, PlaceStrength::STRENGTH_LOCKED);

            // reconnect wires
            // A
            ci.disconnectPort(id_IO);
            in_iob->disconnectPort(id_I);
            ci.movePortTo(id_I, in_iob, id_I);
            ci.movePortTo(id_OH, in_iob, id_O);
            in_iob->disconnectPort(id_OEN);
            ci.movePortTo(id_OEN, in_iob, id_OEN);
            // B
            ci.disconnectPort(id_IO);
            inb_iob->disconnectPort(id_I);
            ci.movePortTo(id_IB, inb_iob, id_I);
            ci.movePortTo(id_OB, inb_iob, id_O);
            inb_iob->disconnectPort(id_OEN);
            ci.movePortTo(id_OENB, inb_iob, id_OEN);
            // MIPI enable (?)
            ci.addInput(ctx->id("MIPIEN0"));
            ci.connectPort(ctx->id("MIPIEN0"), ctx->nets.at(ctx->id("$PACKER_GND")).get());
            ci.addInput(ctx->id("MIPIEN1"));
            ci.connectPort(ctx->id("MIPIEN1"), ctx->nets.at(ctx->id("$PACKER_VCC")).get());

            in_iob->setParam(id_MIPI_IBUF, 1);
            inb_iob->setParam(id_MIPI_IBUF, 1);
        } break;
        default:
            log_error("MIPI %s is not implemented.\n", ci.type.c_str(ctx));
        }
    }
    for (auto &ncell : new_cells) {
        ctx->cells[ncell->name] = std::move(ncell);
    }
}

void GowinPacker::pack_diff_iobs(void)
{
    log_info("Pack diff IOBs...\n");
    std::vector<IdString> cells_to_remove, nets_to_remove;

    for (auto &cell : ctx->cells) {
        CellInfo &ci = *cell.second;
        if (!is_diffio(&ci)) {
            continue;
        }
        if (!gwu.is_diff_io_supported(ci.type)) {
            log_error("%s is not supported\n", ci.type.c_str(ctx));
        }
        cells_to_remove.push_back(ci.name);
        auto pn_cells = get_pn_cells(ci);
        NPNR_ASSERT(pn_cells.first != nullptr && pn_cells.second != nullptr);

        mark_iobs_as_diff(ci, pn_cells);
        switch_diff_ports(ci, pn_cells, nets_to_remove);
    }

    for (auto cell : cells_to_remove) {
        ctx->cells.erase(cell);
    }
    for (auto net : nets_to_remove) {
        ctx->nets.erase(net);
    }
}

static bool is_ff(const Context *ctx, CellInfo *cell) { return is_dff(cell); }

static bool is_gw5a25a(const Context *ctx)
{
    IdString chipdb_key = ctx->id("packer.chipdb");
    if (!ctx->settings.count(chipdb_key))
        return false;
    std::string family = ctx->settings.at(chipdb_key).as_string();
    return family.rfind("GW5A-25", 0) == 0;
}

static bool r56_env_enabled(const char *name)
{
    const char *value = getenv(name);
    if (value == nullptr || value[0] == '\0')
        return false;
    return strcmp(value, "0") != 0 && strcmp(value, "false") != 0 && strcmp(value, "FALSE") != 0 &&
           strcmp(value, "off") != 0 && strcmp(value, "OFF") != 0 && strcmp(value, "no") != 0 &&
           strcmp(value, "NO") != 0;
}

// Default-ON gate (opt-out only): true unless the env var is explicitly
// set to a falsey value. Used for proven, baked-in fixes (mirrors the
// gowin.cc r57_env_disabled / GOWIN_BUFFER_LUTS default-ON policy).
static bool r56_env_default_on(const char *name)
{
    const char *value = getenv(name);
    if (value == nullptr || value[0] == '\0')
        return true;
    return !(strcmp(value, "0") == 0 || strcmp(value, "false") == 0 || strcmp(value, "FALSE") == 0 ||
             strcmp(value, "off") == 0 || strcmp(value, "OFF") == 0 || strcmp(value, "no") == 0 ||
             strcmp(value, "NO") == 0);
}

static bool is_sdram_dq_iobuf(const Context *ctx, const CellInfo &ci)
{
    if (ci.type != id_IOBUF)
        return false;
    std::string name = ci.name.str(ctx);
    if (name.find("gen_sdram_dq_iob[") != std::string::npos &&
        name.find(".u_sdram_dq_iobuf") != std::string::npos)
        return true;
    return name.find("$iopadmap$") != std::string::npos &&
           name.find(".IO_sdram_dq[") != std::string::npos;
}

// Narrow detector for DQ[12] only (re-added 2026-05-21 for the placement-
// perturbation experiment after IOLOGIC OREG path produced 3 HW failures).
// Codex (job 6c55f8d5) recommends pivoting to baseline-only DQ[12] capture
// FF placement experiments. Bug is DQ[12] reads INVERTED in baseline
// (12:00EF7E80 vs Gowin 12:10EF6E80). Default fabric placement puts the
// capture FF at X3Y34/DFF6 which appears to sample at a marginal SDRAM
// read-window edge.
static bool is_sdram_dq12_iobuf(const Context *ctx, const CellInfo &ci)
{
    if (ci.type != id_IOBUF)
        return false;
    std::string name = ci.name.str(ctx);
    return name.find(".IO_sdram_dq[12]") != std::string::npos ||
           (name.find("gen_sdram_dq_iob[12]") != std::string::npos &&
            name.find(".u_sdram_dq_iobuf") != std::string::npos);
}

static bool is_r56_fabric_dq_iobuf(const Context *ctx, const CellInfo &ci)
{
    // R56 "Path-B" (suppress the unrealizable GW5A pad-IOLOGIC input
    // register, keep the SDRAM-DQ capture FF in fabric; r76G then forces
    // it onto the main clock spine) is the ONLY proven-correct SDRAM-DQ
    // read realization on the OSS GW5A-25A toolchain — the r46 fingerprint
    // proved the pad-IOLOGIC alternative is a dead end (apicula cannot
    // synthesize the full registered-EMPTY-IOLOGIC fuse set). The old
    // R56_DQ_PATHB env selector split the test harness (=1, where the fix
    // was proven) from the product EXP_HH (=0, where the fix never ran).
    // Collapsed 2026-05-20 (user-authorized): Path-B is now UNCONDITIONAL
    // for any GW5A-25A SDRAM-DQ IOBUF, so the minimal harness and the
    // EXP_HH loader build the IDENTICAL proven path. (Intentionally
    // changes the EXP_HH/r35 bitstream off the legacy buffer-LUT md5
    // 29359f54 onto the proven Path-B — pending HW re-validation.)
    if (!is_gw5a25a(ctx) || ci.type != id_IOBUF)
        return false;
    std::string name = ci.name.str(ctx);
    return is_sdram_dq_iobuf(ctx, ci) ||
           (name.find("gen_bidir_iob[") != std::string::npos &&
            name.find(".u_bidir_iobuf") != std::string::npos);
}

static bool incompatible_ffs(IdString type_a, IdString type_b)
{
    return type_a != type_b &&
           ((type_a == id_DFFS && type_b != id_DFFR) || (type_a == id_DFFR && type_b != id_DFFS) ||
            (type_a == id_DFFSE && type_b != id_DFFRE) || (type_a == id_DFFRE && type_b != id_DFFSE) ||
            (type_a == id_DFFP && type_b != id_DFFC) || (type_a == id_DFFC && type_b != id_DFFP) ||
            (type_a == id_DFFPE && type_b != id_DFFCE) || (type_a == id_DFFCE && type_b != id_DFFPE) ||
            (type_a == id_DFFNS && type_b != id_DFFNR) || (type_a == id_DFFNR && type_b != id_DFFNS) ||
            (type_a == id_DFFNSE && type_b != id_DFFNRE) || (type_a == id_DFFNRE && type_b != id_DFFNSE) ||
            (type_a == id_DFFNP && type_b != id_DFFNC) || (type_a == id_DFFNC && type_b != id_DFFNP) ||
            (type_a == id_DFFNPE && type_b != id_DFFNCE) || (type_a == id_DFFNCE && type_b != id_DFFNPE) ||
            (type_a == id_DFF && type_b != id_DFF) || (type_a == id_DFFN && type_b != id_DFFN) ||
            (type_a == id_DFFE && type_b != id_DFFE) || (type_a == id_DFFNE && type_b != id_DFFNE));
}

void GowinPacker::pack_io_regs(void)
{
    log_info("Pack FFs into IO cells...\n");
    std::vector<IdString> cells_to_remove;
    std::vector<IdString> nets_to_remove;
    std::vector<std::unique_ptr<CellInfo>> new_cells;

    for (auto &cell : ctx->cells) {
        CellInfo &ci = *cell.second;
        if (!is_io(&ci)) {
            continue;
        }
        if (ci.attrs.count(id_NOIOBFF)) {
            if (ctx->debug) {
                log_info(" NOIOBFF attribute at %s. Skipping FF placement.\n", ctx->nameOf(&ci));
            }
            continue;
        }

        // EXP_HH DQ[12] placement-perturbation probe (codex job 6c55f8d5,
        // 2026-05-21). Env-gated by EXP_HH_DQ12_PLACE_BEL=<bel_name>, e.g.
        // EXP_HH_DQ12_PLACE_BEL=X3Y34/DFF4 to lock the capture FF to a
        // DIFFERENT slot pair in the same tile. Default OFF -> baseline
        // 2eb1dd7b reproduces unchanged.
        if (is_sdram_dq12_iobuf(ctx, ci) && ci.getPort(id_O) != nullptr) {
            const char *target_bel_str = getenv("EXP_HH_DQ12_PLACE_BEL");
            if (target_bel_str != nullptr && target_bel_str[0] != '\0') {
                NetInfo *o_net = ci.ports.at(id_O).net;
                if (o_net != nullptr) {
                    CellInfo *target_ff = nullptr;
                    for (auto &usr : o_net->users) {
                        CellInfo *u = usr.cell;
                        if (u == nullptr)
                            continue;
                        if (u->type.in(id_DFFCE, id_DFFRE, id_DFFE, id_DFF, id_DFFSE, id_DFFPE) &&
                            usr.port == id_D) {
                            target_ff = u;
                            break;
                        }
                        // hop through a single LUT4 buffer (BUFLUT or R76G msink)
                        if (u->type == id_LUT4 && (usr.port == id_I0 || usr.port == id_I3)) {
                            NetInfo *f_net = u->getPort(id_F);
                            if (f_net != nullptr) {
                                for (auto &usr2 : f_net->users) {
                                    if (usr2.cell != nullptr &&
                                        usr2.cell->type.in(id_DFFCE, id_DFFRE, id_DFFE, id_DFF,
                                                            id_DFFSE, id_DFFPE) &&
                                        usr2.port == id_D) {
                                        target_ff = usr2.cell;
                                        break;
                                    }
                                }
                            }
                        }
                        if (target_ff != nullptr)
                            break;
                    }
                    if (target_ff != nullptr) {
                        BelId target_bel = ctx->getBelByNameStr(target_bel_str);
                        if (target_bel == BelId()) {
                            log_warning("EXP_HH_DQ12_PLACE_BEL=%s: BEL not found in chipdb; skipping.\n",
                                        target_bel_str);
                        } else if (!ctx->checkBelAvail(target_bel)) {
                            log_warning("EXP_HH_DQ12_PLACE_BEL=%s: %s already taken by %s; skipping.\n",
                                        target_bel_str, target_bel_str,
                                        ctx->nameOf(ctx->getBoundBelCell(target_bel)));
                        } else if (target_ff->bel != BelId()) {
                            log_warning("EXP_HH_DQ12_PLACE_BEL=%s: target FF %s already bound to %s; "
                                        "skipping.\n",
                                        target_bel_str, ctx->nameOf(target_ff),
                                        ctx->nameOfBel(target_ff->bel));
                        } else {
                            ctx->bindBel(target_bel, target_ff, PlaceStrength::STRENGTH_LOCKED);
                            log_info("  EXP_HH_DQ12_PLACE_BEL=%s: locked DQ[12] capture FF %s to %s "
                                     "(bindBel).\n",
                                     target_bel_str, ctx->nameOf(target_ff), target_bel_str);
                        }
                    } else {
                        log_warning("EXP_HH_DQ12_PLACE_BEL=%s: could not find a downstream DFF for %s.\n",
                                    target_bel_str, ctx->nameOf(&ci));
                    }
                }
            }
        }

        // In the case of placing multiple registers in the IO it should be
        // noted that the CLK, ClockEnable and LocalSetReset nets must
        // match.
        const NetInfo *clk_net = nullptr;
        const NetInfo *ce_net = nullptr;
        const NetInfo *lsr_net = nullptr;
        IdString reg_type;

        // input reg in IO
        CellInfo *iologic_i = nullptr;
        bool r56_keep_fabric_dq_input = false;

        // EXP_HH DQ[12] IOLOGIC input-register migration (codex job
        // 9113ce1b, 2026-05-22). Env-gated by EXP_HH_DQ12_IOLOGIC=1,
        // default OFF -> baseline 2eb1dd7b reproduces byte-identical.
        //
        // Mission: SDRAM-DQ[12] reads INVERTED on the OSS toolchain
        // (firmware-stream 12:00EF7E80 vs the proven Gowin EDA twin's
        // 12:10EF6E80 -> READY). The Gowin .vg RE (job 9113ce1b) showed
        // Gowin's synthesis emits a plain TBUF for every SDRAM-DQ pin;
        // the IOLOGIC input register at R37C4 is produced by Gowin's
        // PLACER auto-packing the fabric capture FF into the IOB. OSS
        // leaves that FF in a fabric DFFCE (LOGIC tile X3Y35) -> wrong
        // sample phase -> inverted read.
        //
        // The 3 prior IOLOGIC HW failures (md5 09fc2470/1e923f48/eed1fedd,
        // dead silicon) happened because apicula's GW5A IOLOGIC fuse
        // vocabulary was only ~32/141 complete: nextpnr requested an
        // IOLOGIC register, apicula encoded it with MISSING fuses ->
        // electrically-underspecified IOB. The 2026-05 DB-vocab expansion
        // (overlay v7 = 102/141, and 21/21 for DQ[12]) + the GW5A IODELAY
        // encoder fix (apicula 834622f) closed that gap, so apicula can
        // now write the complete IOLOGICI_EMPTY+HAS_REG fuse set.
        //
        // Scope: DQ[12] ONLY, INPUT register ONLY (no OREG/TREG migration).
        // Placed BEFORE the r76G synthetic-multi-sink block so the
        // single-fanout O net is intact for net_only_drives(). On success
        // the fabric FF becomes an IOLOGICI_EMPTY cell (no longer is_ff),
        // so the r76G / r56 / generic-806 / generic-868 blocks below all
        // naturally skip DQ[12] -> no edits to those blocks are needed.
        bool dq12_iologic_migrated = false;
        if (is_sdram_dq12_iobuf(ctx, ci) && r56_env_enabled("EXP_HH_DQ12_IOLOGIC") &&
            ci.getPort(id_O) != nullptr) {
            NetInfo *o_net = ci.ports.at(id_O).net;
            CellInfo *ff = (o_net != nullptr) ? net_only_drives(ctx, o_net, is_ff, id_D) : nullptr;
            if (ff == nullptr) {
                log_warning("EXP_HH_DQ12_IOLOGIC: DQ[12] O net has no single fabric capture "
                            "FF; skipping migration (baseline preserved).\n");
            } else if (o_net->users.entries() != 1) {
                log_warning("EXP_HH_DQ12_IOLOGIC: DQ[12] O net is multi-sink; skipping "
                            "migration (baseline preserved).\n");
            } else {
                BelId l_bel = get_iologici_bel(&ci);
                if (l_bel == BelId()) {
                    log_warning("EXP_HH_DQ12_IOLOGIC: no IOLOGICI bel for DQ[12]; skipping.\n");
                } else {
                    std::string ff_type = ff->type.str(ctx);
                    IdString iologic_name = gwu.create_aux_name(ci.name, 0, "_dq12_iobff$");
                    auto iologic_cell = gwu.create_cell(iologic_name, id_IOLOGICI_EMPTY);
                    new_cells.push_back(std::move(iologic_cell));
                    CellInfo *dq12_iologic = new_cells.back().get();
                    for (auto &port : ff->ports) {
                        IdString port_name = port.first;
                        ff->movePortTo(port_name, dq12_iologic,
                                       port_name != id_Q ? port_name : id_Q4);
                    }
                    dq12_iologic->setAttr(id_HAS_REG, 1);
                    dq12_iologic->setAttr(id_IREG_TYPE, ff_type);
                    cells_to_remove.push_back(ff->name);
                    dq12_iologic_migrated = true;
                    log_info("  EXP_HH_DQ12_IOLOGIC: migrated DQ[12] capture FF %s into "
                             "IOLOGICI_EMPTY %s (HAS_REG=1, IREG_TYPE=%s) -- input register "
                             "only, no OREG/TREG.\n",
                             ctx->nameOf(ff), ctx->nameOf(dq12_iologic), ff_type.c_str());
                    // Option 2A++: optional IODELAY tap on the IOLOGIC cell.
                    // Combines iter9 fuse fix (bit-28 fix) with variable input-delay tap
                    // to potentially recover bit-12 (beat-A) sampling.
                    const char *iodly_env = getenv("EXP_HH_DQ12_IOLOGIC_IODELAY");
                    if (iodly_env != nullptr && *iodly_env != '\0') {
                        int dly_val = atoi(iodly_env);
                        if (dly_val < 0) dly_val = 0;
                        if (dly_val > 127) dly_val = 127;
                        dq12_iologic->setAttr(id_IODELAY, Property("IN"));
                        dq12_iologic->setParam(id_C_STATIC_DLY, Property(dly_val, 32));
                        log_info("  EXP_HH_DQ12_IOLOGIC_IODELAY=%d: added IODELAY=IN with "
                                 "C_STATIC_DLY=%d to %s.\n",
                                 dly_val, dly_val, ctx->nameOf(dq12_iologic));
                    }
                }
            }
        }
        (void)dq12_iologic_migrated;

        // R76G: env-gated synthetic multi-sink. Root cause (this session,
        // multi-artifact + Codex-corroborated): for a SINGLE-fanout
        // SDRAM-DQ IOBUF.O, nextpnr deterministically routes the capture
        // FF clock from the IOB-column GBxx global branch (apicula .fs:
        // DQ-tile CLK0=GBxx) at a wrong effective phase -> deterministic
        // RD=F000 (seed-invariant). The ONLY structure proven to PASS on
        // HW is the MULTI-fanout IOBUF.O of r70P (.fs CLK0=VCC, main
        // spine). Placement levers (r75 lock-off, r76F v1 leave-to-placer,
        // v3 force-far) ALL still produced CLK0=GBxx -> placement is not
        // the lever; net fanout is. So when R56_DQ_FABRIC_AWAY is enabled
        // we add a kept 2nd sink (LUT4 buffer) on the single-fanout DQ
        // IOBUF.O net, exactly reproducing the proven-good r70P Heisenbug
        // at the netlist level: the net becomes multi-fanout, the
        // users.entries()==1 R56-tag/near-IOB path below is NOT taken
        // (same as r70P), and the router clocks the capture FF off the
        // main spine. BAKED-IN DEFAULT-ON (proven HW fix; opt out with
        // R56_DQ_FABRIC_AWAY=0). r35/EXP_HH has R56_DQ_PATHB=0 so
        // is_r56_fabric_dq_iobuf == false -> this whole block is
        // unreachable for EXP_HH -> bitstream byte-identical (md5
        // 29359f54) regardless of the default.
        if (r56_env_default_on("R56_DQ_FABRIC_AWAY") && is_r56_fabric_dq_iobuf(ctx, ci) &&
            ci.getPort(id_O) != nullptr) {
            NetInfo *o_net = ci.ports.at(id_O).net;
            if (o_net != nullptr && o_net->users.entries() == 1 &&
                net_only_drives(ctx, o_net, is_ff, id_D) != nullptr) {
                IdString sink_name = gwu.create_aux_name(ci.name, 0, "_r76g_msink$");
                new_cells.push_back(gwu.create_cell(sink_name, id_LUT4));
                CellInfo *msink = new_cells.back().get();
                msink->addInput(id_I0);
                msink->addOutput(id_F);
                msink->setParam(id_INIT, Property(0xAAAA, 16)); // F = I0 (buffer)
                msink->setAttr(ctx->id("keep"), 1);
                IdString keep_net_name = gwu.create_aux_name(ci.name, 0, "_r76g_keep$");
                NetInfo *keep_net = ctx->createNet(keep_net_name);
                msink->connectPort(id_I0, o_net);   // 2nd user -> multi-fanout
                msink->connectPort(id_F, keep_net);
                log_info("  r76g: synthetic multi-sink LUT on %s O-net (mirror proven-good "
                         "r70P; single-fanout -> multi-fanout so DQ capture clock uses the "
                         "main spine, not the near-IOB GBxx branch).\n",
                         ctx->nameOf(&ci));
            }
        }

        if (is_r56_fabric_dq_iobuf(ctx, ci) && ci.getPort(id_O) != nullptr) {
            CellInfo *ff = net_only_drives(ctx, ci.ports.at(id_O).net, is_ff, id_D);
            if (ff != nullptr && ci.ports.at(id_O).net->users.entries() == 1) {
                ff->setAttr(ctx->id("R56_FABRIC_DQ_CAPTURE"), 1);
                ff->setAttr(ctx->id("R56_DQ_IOB_CELL"), Property(ci.name.str(ctx)));
                r56_keep_fabric_dq_input = true;
                log_info("  r56: keep DQ input FF %s for %s in fabric; placement will lock it near the IOB.\n",
                         ctx->nameOf(ff), ctx->nameOf(&ci));
            } else if (ci.attrs.count(id_IOBFF)) {
                log_warning("r56: DQ input fabric capture requested at %s, but no single fabric FF is driven by O.\n",
                            ctx->nameOf(&ci));
            }
        }

        if (!r56_keep_fabric_dq_input &&
            ((ci.type == id_IBUF && (ctx->settings.count(id_IREG_IN_IOB) || ci.attrs.count(id_IOBFF))) ||
             (ci.type == id_IOBUF && (ctx->settings.count(id_IOREG_IN_IOB) || ci.attrs.count(id_IOBFF))))) {

            if (ci.getPort(id_O) == nullptr) {
                continue;
            }
            // OBUF O -> D FF
            CellInfo *ff = net_only_drives(ctx, ci.ports.at(id_O).net, is_ff, id_D);
            if (ff == nullptr) {
                if (ci.attrs.count(id_IOBFF)) {
                    log_warning("Port O of %s is not connected to FF.\n", ctx->nameOf(&ci));
                }
                continue;
            }
            if (ci.ports.at(id_O).net->users.entries() != 1) {
                if (ci.attrs.count(id_IOBFF)) {
                    log_warning("Port O of %s is the driver of %s multi-sink network.\n", ctx->nameOf(&ci),
                                ctx->nameOf(ci.ports.at(id_O).net));
                }
                continue;
            }
            BelId l_bel = get_iologici_bel(&ci);
            if (l_bel == BelId()) {
                continue;
            }
            if (ctx->debug) {
                log_info(" trying %s ff as Input Register of %s IO\n", ctx->nameOf(ff), ctx->nameOf(&ci));
            }

            clk_net = ff->getPort(id_CLK);
            ce_net = ff->getPort(id_CE);
            for (IdString port : {id_SET, id_RESET, id_PRESET, id_CLEAR}) {
                lsr_net = ff->getPort(port);
                if (lsr_net != nullptr) {
                    break;
                }
            }
            reg_type = ff->type;

            // create IOLOGIC cell for flipflop
            IdString iologic_name = gwu.create_aux_name(ci.name, 0, "_iobff$");
            auto iologic_cell = gwu.create_cell(iologic_name, id_IOLOGICI_EMPTY);
            new_cells.push_back(std::move(iologic_cell));
            iologic_i = new_cells.back().get();

            // move ports
            for (auto &port : ff->ports) {
                IdString port_name = port.first;
                ff->movePortTo(port_name, iologic_i, port_name != id_Q ? port_name : id_Q4);
            }
            if (ctx->verbose) {
                log_info("  place FF %s into IBUF %s, make iologic_i %s\n", ctx->nameOf(ff), ctx->nameOf(&ci),
                         ctx->nameOf(iologic_i));
            }
            iologic_i->setAttr(id_HAS_REG, 1);
            iologic_i->setAttr(id_IREG_TYPE, ff->type.str(ctx));
            cells_to_remove.push_back(ff->name);
        }

        // output reg in IO
        CellInfo *iologic_o = nullptr;
        if ((ci.type == id_OBUF && (ctx->settings.count(id_OREG_IN_IOB) || ci.attrs.count(id_IOBFF))) ||
            (ci.type == id_IOBUF && (ctx->settings.count(id_IOREG_IN_IOB) || ci.attrs.count(id_IOBFF)))) {
            do {
                if (ci.getPort(id_I) == nullptr) {
                    break;
                }
                // OBUF I <- Q FF
                CellInfo *ff = net_driven_by(ctx, ci.ports.at(id_I).net, is_ff, id_Q);
                if (ff == nullptr) {
                    if (ci.attrs.count(id_IOBFF)) {
                        log_warning("Port I of %s is not connected to FF.\n", ctx->nameOf(&ci));
                    }
                } else {
                    if (ci.ports.at(id_I).net->users.entries() != 1) {
                        if (ci.attrs.count(id_IOBFF)) {
                            log_warning("Port I of %s is not the only sink on the %s network.\n", ctx->nameOf(&ci),
                                        ctx->nameOf(ci.ports.at(id_I).net));
                        }
                        break;
                    }
                    BelId l_bel = get_iologico_bel(&ci);
                    if (l_bel == BelId()) {
                        break;
                    }

                    const NetInfo *this_clk_net = ff->getPort(id_CLK);
                    const NetInfo *this_ce_net = ff->getPort(id_CE);
                    const NetInfo *this_lsr_net;
                    for (IdString port : {id_SET, id_RESET, id_PRESET, id_CLEAR}) {
                        this_lsr_net = ff->getPort(port);
                        if (this_lsr_net != nullptr) {
                            break;
                        }
                    }
                    // The IOBUF may already have registers placed.  The input
                    // and output registers live in separate IOLOGIC halves, so
                    // they do not share the slice FF type/control-set
                    // restrictions modelled by incompatible_ffs().
                    if (ci.type == id_IOBUF) {
                        if (iologic_i != nullptr) {
                            // No compatibility check here: IREG and OREG are
                            // independent hardware paths on a bidirectional IO.
                        } else {
                            clk_net = this_clk_net;
                            ce_net = this_ce_net;
                            lsr_net = this_lsr_net;
                            reg_type = ff->type;
                        }
                    }

                    // create IOLOGIC cell for flipflop
                    IdString iologic_name = gwu.create_aux_name(ci.name, 1, "_iobff$");
                    auto iologic_cell = gwu.create_cell(iologic_name, id_IOLOGICO_EMPTY);
                    new_cells.push_back(std::move(iologic_cell));
                    iologic_o = new_cells.back().get();

                    // move ports
                    for (auto &port : ff->ports) {
                        IdString port_name = port.first;
                        ff->movePortTo(port_name, iologic_o, port_name != id_D ? port_name : id_D0);
                    }
                    if (ctx->verbose) {
                        log_info("  place FF %s into OBUF %s, make iologic_o %s\n", ctx->nameOf(ff), ctx->nameOf(&ci),
                                 ctx->nameOf(iologic_o));
                    }
                    iologic_o->setAttr(id_HAS_REG, 1);
                    iologic_o->setAttr(id_OREG_TYPE, ff->type.str(ctx));
                    cells_to_remove.push_back(ff->name);
                }
            } while (false);
        }

        // output enable reg in IO
        if (ci.type == id_IOBUF && (ctx->settings.count(id_IOREG_IN_IOB) || ci.attrs.count(id_IOBFF))) {
            do {
                if (ci.getPort(id_OEN) == nullptr) {
                    break;
                }
                // IOBUF OEN <- Q FF
                CellInfo *ff = net_driven_by(ctx, ci.ports.at(id_OEN).net, is_ff, id_Q);
                if (ff != nullptr) {
                    if (ci.ports.at(id_OEN).net->users.entries() != 1) {
                        if (ci.attrs.count(id_IOBFF)) {
                            log_warning("Port OEN of %s is not the only sink on the %s network.\n", ctx->nameOf(&ci),
                                        ctx->nameOf(ci.ports.at(id_OEN).net));
                        }
                        break;
                    }
                    BelId l_bel = get_iologico_bel(&ci);
                    if (l_bel == BelId()) {
                        break;
                    }
                    if (ctx->debug) {
                        log_info(" trying %s ff as Output Enable Register of %s IO\n", ctx->nameOf(ff),
                                 ctx->nameOf(&ci));
                    }

                    const NetInfo *this_clk_net = ff->getPort(id_CLK);
                    const NetInfo *this_ce_net = ff->getPort(id_CE);
                    const NetInfo *this_lsr_net;
                    for (IdString port : {id_SET, id_RESET, id_PRESET, id_CLEAR}) {
                        this_lsr_net = ff->getPort(port);
                        if (this_lsr_net != nullptr) {
                            break;
                        }
                    }

                    // The IOBUF may already have registers placed
                    if (iologic_i != nullptr || iologic_o != nullptr) {
                        if (iologic_o == nullptr) {
                            iologic_o = iologic_i;
                        }
                        if (incompatible_ffs(ff->type, reg_type)) {
                            if (ci.attrs.count(id_IOBFF)) {
                                log_warning("TREG type conflict:%s:%s vs %s IREG/OREG:%s\n", ctx->nameOf(ff),
                                            ff->type.c_str(ctx), ctx->nameOf(&ci), reg_type.c_str(ctx));
                            }
                            break;
                        } else {
                            if (clk_net != this_clk_net || ce_net != this_ce_net || lsr_net != this_lsr_net) {
                                if (clk_net != this_clk_net) {
                                    if (ci.attrs.count(id_IOBFF)) {
                                        log_warning("Conflicting TREG CLK nets at %s:'%s' vs '%s'\n", ctx->nameOf(&ci),
                                                    ctx->nameOf(clk_net), ctx->nameOf(this_clk_net));
                                    }
                                }
                                if (ce_net != this_ce_net) {
                                    if (ci.attrs.count(id_IOBFF)) {
                                        log_warning("Conflicting TREG CE nets at %s:'%s' vs '%s'\n", ctx->nameOf(&ci),
                                                    ctx->nameOf(ce_net), ctx->nameOf(this_ce_net));
                                    }
                                }
                                if (lsr_net != this_lsr_net) {
                                    if (ci.attrs.count(id_IOBFF)) {
                                        log_warning("Conflicting TREG LSR nets at %s:'%s' vs '%s'\n", ctx->nameOf(&ci),
                                                    ctx->nameOf(lsr_net), ctx->nameOf(this_lsr_net));
                                    }
                                }
                                break;
                            }
                        }
                    }

                    if (iologic_o == nullptr) {
                        // create IOLOGIC cell for flipflop
                        IdString iologic_name = gwu.create_aux_name(ci.name, 2, "_iobff$");
                        auto iologic_cell = gwu.create_cell(iologic_name, id_IOLOGICO_EMPTY);
                        new_cells.push_back(std::move(iologic_cell));
                        iologic_o = new_cells.back().get();
                    }

                    // move ports
                    for (auto &port : ff->ports) {
                        IdString port_name = port.first;
                        if (port_name == id_Q) {
                            continue;
                        }
                        ff->movePortTo(port_name, iologic_o, port_name != id_D ? port_name : id_TX);
                    }

                    nets_to_remove.push_back(ci.getPort(id_OEN)->name);
                    ci.disconnectPort(id_OEN);
                    ff->disconnectPort(id_Q);

                    if (ctx->verbose) {
                        log_info("  place FF %s into IOBUF %s, make iologic_o %s\n", ctx->nameOf(ff), ctx->nameOf(&ci),
                                 ctx->nameOf(iologic_o));
                    }
                    iologic_o->setAttr(id_HAS_REG, 1);
                    iologic_o->setAttr(id_TREG_TYPE, ff->type.str(ctx));
                    cells_to_remove.push_back(ff->name);
                }
            } while (false);
        }
    }

    for (auto cell : cells_to_remove) {
        ctx->cells.erase(cell);
    }

    for (auto &ncell : new_cells) {
        ctx->cells[ncell->name] = std::move(ncell);
    }

    for (auto net : nets_to_remove) {
        ctx->nets.erase(net);
    }
}

NEXTPNR_NAMESPACE_END
