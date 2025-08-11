`default_nettype none
`timescale 1ns/1ps

// 简化版 CAN 控制器寄存器外壳 + 自环(loopback)验证路径
// - Wishbone ACK 单拍应答（classic，零等待）
// - STATUS 寄存器采用 W1C(Write-1-to-Clear) 语义
// - CMD 位语义明确：bit0=TX_START(只读为0，自清脉冲)，bit1=LOOPBACK_EN
// - 地址对齐：按 word 地址 wb_adr_i[5:2] 解码
// - 自环：写入 TX_* 后，写 CMD.TX_START=1 -> 置位 TX_DONE，若 LOOPBACK_EN=1 同步置 RX_READY 并回填 RX_*

module simple_can_ctrl(
    input  wire        clk,
    input  wire        rst_n,

    // Wishbone (精简)接口
    input  wire [31:0] wb_adr_i,
    input  wire [31:0] wb_dat_i,
    output reg  [31:0] wb_dat_o,
    input  wire        wb_we_i,
    input  wire        wb_cyc_i,
    input  wire        wb_stb_i,
    output reg         wb_ack_o,

    // 便于上层观察（示例输出）
    output reg         rx_ready,      // = STATUS[1]
    output reg  [10:0] rx_id,
    output reg  [7:0]  rx_data0,
    output reg  [7:0]  rx_data1
);

    // ----------------------
    // 寄存器映射（word 地址）
    // ----------------------
    localparam [3:0] ADDR_CMD       = 4'h0; // 0x00
    localparam [3:0] ADDR_STATUS    = 4'h1; // 0x04
    localparam [3:0] ADDR_TX_ID     = 4'h2; // 0x08
    localparam [3:0] ADDR_TX_LEN    = 4'h3; // 0x0C
    localparam [3:0] ADDR_TX_DATA0  = 4'h4; // 0x10
    localparam [3:0] ADDR_TX_DATA1  = 4'h5; // 0x14
    localparam [3:0] ADDR_RX_ID     = 4'h6; // 0x18
    localparam [3:0] ADDR_RX_LEN    = 4'h7; // 0x1C
    localparam [3:0] ADDR_RX_DATA0  = 4'h8; // 0x20
    localparam [3:0] ADDR_RX_DATA1  = 4'h9; // 0x24

    // CMD 位定义
    // bit0: TX_START (写1触发发送；读时恒为0；硬件打一拍脉冲)
    // bit1: LOOPBACK_EN (回环开关)
    reg        loopback_en;
    reg        tx_start_pulse; // 单拍脉冲（由写CMD触发）

    // STATUS 位定义（W1C）
    // bit0: TX_DONE
    // bit1: RX_READY
    // bit2: ERR（预留）
    reg  [31:0] status_reg;

    // 发送侧寄存器
    reg  [10:0] tx_id;
    reg  [7:0]  tx_len;      // DLC 0..8
    reg  [31:0] tx_data0;    // 低8位有效
    reg  [31:0] tx_data1;    // 低8位有效

    // 接收侧寄存器
    reg  [7:0]  rx_len;      // DLC

    // Wishbone 访问信号
    wire        wb_access = wb_cyc_i & wb_stb_i;
    wire        wb_write  = wb_access &  wb_we_i;
    wire        wb_read   = wb_access & ~wb_we_i;
    wire [3:0]  waddr     = wb_adr_i[5:2];

    // ----------------------
    // 同步时序
    // ----------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wb_ack_o       <= 1'b0;
            wb_dat_o       <= 32'd0;

            loopback_en    <= 1'b0;
            tx_start_pulse <= 1'b0;

            status_reg     <= 32'd0;

            tx_id          <= 11'd0;
            tx_len         <= 8'd0;
            tx_data0       <= 32'd0;
            tx_data1       <= 32'd0;

            rx_id          <= 11'd0;
            rx_len         <= 8'd0;
            rx_data0       <= 8'd0;
            rx_data1       <= 8'd0;

            rx_ready       <= 1'b0;

        end else begin
            // 默认单拍控制清零
            tx_start_pulse <= 1'b0;

            // Wishbone ACK：当有访问且当前未应答时，打一拍 ACK
            if (wb_access & ~wb_ack_o)
                wb_ack_o <= 1'b1;
            else
                wb_ack_o <= 1'b0;

            // ==========================
            // 写访问（在 ACK 拍执行）
            // ==========================
            if (wb_write & ~wb_ack_o) begin
                case (waddr)
                    ADDR_CMD: begin
                        loopback_en    <= wb_dat_i[1];
                        if (wb_dat_i[0])    // 写1触发
                            tx_start_pulse <= 1'b1; // 仅此拍为1
                        // 注意：读 CMD 时返回 {..,loopback_en,1'b0}
                    end
                    ADDR_STATUS: begin
                        // W1C：写1清零相应状态位
                        if (wb_dat_i[0]) status_reg[0] <= 1'b0; // TX_DONE
                        if (wb_dat_i[1]) status_reg[1] <= 1'b0; // RX_READY
                        if (wb_dat_i[2]) status_reg[2] <= 1'b0; // ERR
                    end
                    ADDR_TX_ID:     tx_id    <= wb_dat_i[10:0];
                    ADDR_TX_LEN:    tx_len   <= wb_dat_i[7:0];
                    ADDR_TX_DATA0:  tx_data0 <= wb_dat_i;
                    ADDR_TX_DATA1:  tx_data1 <= wb_dat_i;
                    default: /* no write */ ;
                endcase
            end

            // ==========================
            // 读访问（在 ACK 拍返回数据）
            // ==========================
            if (wb_read & ~wb_ack_o) begin
                case (waddr)
                    ADDR_CMD:      wb_dat_o <= {30'd0, loopback_en, 1'b0}; // TX_START 读为0
                    ADDR_STATUS:   wb_dat_o <= status_reg;
                    ADDR_TX_ID:    wb_dat_o <= {21'd0, tx_id};
                    ADDR_TX_LEN:   wb_dat_o <= {24'd0, tx_len};
                    ADDR_TX_DATA0: wb_dat_o <= tx_data0;
                    ADDR_TX_DATA1: wb_dat_o <= tx_data1;
                    ADDR_RX_ID:    wb_dat_o <= {21'd0, rx_id};
                    ADDR_RX_LEN:   wb_dat_o <= {24'd0, rx_len};
                    ADDR_RX_DATA0: wb_dat_o <= {24'd0, rx_data0};
                    ADDR_RX_DATA1: wb_dat_o <= {24'd0, rx_data1};
                    default:       wb_dat_o <= 32'd0;
                endcase
            end

            // ==========================
            // 命令触发（发送开始）
            // ==========================
            if (tx_start_pulse) begin
                // 目前做最小可用路径：立即完成发送
                status_reg[0] <= 1'b1; // TX_DONE=1

                if (loopback_en) begin
                    // 回环：把 TX 写回 RX
                    rx_id    <= tx_id;
                    rx_len   <= tx_len;
                    rx_data0 <= tx_data0[7:0];
                    rx_data1 <= tx_data1[7:0];
                    status_reg[1] <= 1'b1; // RX_READY=1
                end
            end

            // 输出：保持与 STATUS 同步，便于上层观察
            rx_ready <= status_reg[1];
        end
    end

endmodule


            

