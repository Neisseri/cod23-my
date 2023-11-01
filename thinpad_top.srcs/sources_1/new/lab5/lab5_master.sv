module lab5_master #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
) (
    input wire clk_i,
    input wire rst_i,

    // TODO: 添加需要的控制信号，例如按键开关？
    input wire [ADDR_WIDTH-1:0] top_adr,
    input wire top_cyc,
    input wire top_stb,
    input wire top_we,
    input wire [DATA_WIDTH/8-1:0] top_sel,
    output reg top_ack,
    input wire [DATA_WIDTH-1:0] top_dat_i,
    output reg [DATA_WIDTH-1:0] top_dat_o,

    // wishbone master
    output reg wb_cyc_o,
    output reg wb_stb_o,
    input wire wb_ack_i,
    output reg [ADDR_WIDTH-1:0] wb_adr_o,
    output reg [DATA_WIDTH-1:0] wb_dat_o,
    input wire [DATA_WIDTH-1:0] wb_dat_i,
    output reg [DATA_WIDTH/8-1:0] wb_sel_o,
    output reg wb_we_o
);

  // TODO: 实现实验 5 的内存+串口 Master
  typedef enum logic [1:0] {
    STATE_IDLE = 0,
    STATE_ACTION = 1,
    STATE_DONE = 2
  } state_t;

  state_t state;

  always_ff @ (posedge clk_i) begin

    if (rst_i) begin
      wb_cyc_o <= 1'b0;
      wb_stb_o <= 1'b0;
      wb_adr_o <= 32'b0;
      wb_dat_o <= 32'b0;
      wb_sel_o <= 4'b0;
      wb_we_o <= 1'b0;
      top_ack <= 1'b0;
      top_dat_o <= 32'b0;
      state <= STATE_IDLE;
    end else begin

      case (state)

        STATE_IDLE: begin    
          if (top_stb == 1'b1 && top_cyc == 1'b1) begin // want to send a request
            wb_stb_o <= 1'b1;
            wb_cyc_o <= 1'b1;
            wb_adr_o <= top_adr;
            wb_we_o <= top_we;
            wb_sel_o <= top_sel;
            if (top_we == 1'b1) begin // write
              wb_dat_o <= top_dat_i;
            end
            state <= STATE_ACTION;
          end
        end

        STATE_ACTION: begin
          if (wb_ack_i == 1'b1) begin // wait for slave's operation
            wb_stb_o <= 1'b0;
            wb_cyc_o <= 1'b0;
            top_ack <= 1'b1;
            if (top_we == 1'b0) begin // read
              top_dat_o <= wb_dat_i;
            end
            state <= STATE_DONE;
          end
        end

        STATE_DONE: begin
          wb_cyc_o <= 1'b0;
          wb_stb_o <= 1'b0;
          wb_adr_o <= 32'b0;
          wb_dat_o <= 32'b0;
          wb_sel_o <= 4'b0;
          wb_we_o <= 1'b0;
          top_ack <= 1'b0;
          state <= STATE_IDLE;
        end

        default: begin
          state <= STATE_IDLE;
        end

      endcase
    end
  end

endmodule
