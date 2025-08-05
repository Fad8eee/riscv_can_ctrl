`timescale 1ns/1ps

module simple_can_ctrl(
    input wire clk,
    input wire rst_n,
    input wire [31:0] wb_adr_i,
    input wire [31:0] wb_dat_i,
    output reg [31:0] wb_dat_o,
    input wire        wb_we_i,
    input wire        wb_cyc_i,
    input wire        wb_stb_i,
    output reg        wb_ack_o,

    output reg        rx_ready,
    output reg [10:0] rx_id,
    output reg [7:0]  rx_data0,
    output reg [7:0]  rx_data1
);
reg [31:0] cmd_reg;
reg [31:0] status_reg;
reg [10:0] tx_id;
reg [7:0]  tx_len;
reg [31:0] tx_data0;
reg [31:0] tx_data1;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wb_ack_o   <= 0;
        wb_dat_o   <= 0;
        cmd_reg    <= 0;
        status_reg <= 0;
        tx_id      <= 0;
        tx_len     <= 0;
        tx_data0   <= 0;
        tx_data1   <= 0;
        rx_ready   <= 0;
        rx_id      <= 0;
        rx_data0   <= 0;
        rx_data1   <= 0;
    end else begin
        wb_ack_o   <= 0;
        if (wb_cyc_i && wb_stb_i && !wb_ack_o) begin
            wb_ack_o <= 1;
            if (wb_we_i) begin
                case (wb_adr_i[5:2])
                4'h0:cmd_reg  <= wb_dat_i;
                4'h2:tx_id    <= wb_dat_i[10:0];
                4'h3:tx_len   <= wb_dat_i[7:0];
                4'h4:tx_data0 <= wb_dat_i;
                4'h5:tx_data1 <= wb_dat_i;
                default:/* do nothing */;
            endcase
            end else begin
                case (wb_adr_i[5:2])
                4'h0:wb_dat_o <= cmd_reg;
                4'h1:wb_dat_o <= status_reg;
                4'h2:wb_dat_o <= {21'b0,tx_id};
                4'h6:wb_dat_o <= {21'b0,rx_id};
                4'h7:wb_dat_o <= 8;
                4'h8:wb_dat_o <= {24'b0,rx_data0};
                4'h9:wb_dat_o <= {24'b0,rx_data1};
                default:wb_dat_o <= 0;
            endcase
        end
    end
    if(cmd_reg == 32'h02)begin
        rx_ready   <= 1;
        rx_id      <= tx_id;
        rx_data0   <= tx_data0[7:0];
        rx_data1   <= tx_data1[7:0];
        status_reg <= 32'h1;
        cmd_reg    <= 0;
    end

    if(!wb_we_i && wb_cyc_i && wb_stb_i && (wb_adr_i[5:2] == 4'h7))begin
        status_reg <= 0;
        rx_ready   <= 0;
    end
    end
end
endmodule

            