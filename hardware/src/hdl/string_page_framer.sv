`timescale 1ns / 1ps

`include "libstf_macros.svh"

import lynxTypes::AXI_DATA_BITS;
import libstf::BYTE_T;
import libstf::data8_t;
import parcore::page_conf_t;

/**
 * Preserve native Parquet string payloads for the FPGA string consumer.
 * A page configuration precedes its decompressed payload and stays captured
 * until the page's last payload handshake; only the final page ends the chunk.
 */
module StringPageFramer #(
    parameter DATABEAT_SIZE = AXI_DATA_BITS / 8
) (
    input logic clk,
    input logic rst_n,

    ready_valid_i.s conf, // #(page_conf_t)
    ndata_i.s in,         // #(data8_t, DATABEAT_SIZE)
    typed_ndata_i.m out   // #(DATABEAT_SIZE), BYTE_T framed page bytes
);

`ASSERT_ELAB(DATABEAT_SIZE == 64)

typedef enum logic [1:0] {
    WAIT_CONF,
    EMIT_HEADER,
    EMIT_PAYLOAD
} state_t;

state_t state;
page_conf_t page;
logic [DATABEAT_SIZE * 8 - 1:0] header;

assign conf.ready = state == WAIT_CONF;
assign in.ready = state == EMIT_PAYLOAD && out.ready;

always_comb begin
    header = '0;
    header[31:0] = 32'h31525453;
    header[63:32] = 32'(page.page_type);
    header[95:64] = page.num_values;
    header[127:96] = page.uncompressed_size;
    header[128] = page.last;

    out.data = '0;
    out.typ = BYTE_T;
    out.keep = '0;
    out.last = 1'b0;
    out.valid = 1'b0;

    case (state)
        EMIT_HEADER: begin
            out.data = header;
            out.keep = '1;
            out.valid = 1'b1;
        end
        EMIT_PAYLOAD: begin
            out.data = in.data;
            out.keep = in.keep;
            out.last = in.last && page.last;
            out.valid = in.valid;
        end
        default: begin
        end
    endcase
end

always_ff @(posedge clk) begin
    if (!rst_n) begin
        state <= WAIT_CONF;
        page <= '0;
    end else begin
        case (state)
            WAIT_CONF: begin
                if (conf.valid && conf.ready) begin
                    page <= conf.data;
                    state <= EMIT_HEADER;
                end
            end
            EMIT_HEADER: begin
                if (out.ready) begin
                    state <= EMIT_PAYLOAD;
                end
            end
            EMIT_PAYLOAD: begin
                if (in.valid && in.ready && in.last) begin
                    state <= WAIT_CONF;
                end
            end
            default: begin
                state <= WAIT_CONF;
            end
        endcase
    end
end

endmodule
