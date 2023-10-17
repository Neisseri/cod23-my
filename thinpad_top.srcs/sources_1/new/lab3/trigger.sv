module trigger (
    input wire clk,
    input wire reset,
    input wire push_btn,    // 按键信号
    output reg trigger      // 触发器输出信号
);

    reg last_status = 1'b0;

    // 按键检测模块，在按键上升沿（按下）时设置触发器为高电平
    always_ff @ (posedge clk or posedge reset) begin
        if (reset) begin
            // 复位时将触发器清零
            trigger <= 1'b0;
        end else if (push_btn) begin
            if (last_status == 1'b0) begin
                trigger <= 1'b1;
            end else begin
                trigger <= 1'b0;
            end
            last_status <= 1'b1;
        end else begin // push_btn == 0
            last_status <= 1'b0;
            trigger <= 1'b0;
        end
    end

endmodule
