module button_in (
    input wire clk,
    input wire reset,
    input wire push_btn,
    output reg step
);

    reg last_status = 1'b0;

    always_ff @ (posedge clk) begin
        if (reset) begin
            step <= 1'b0;
            last_status <= 1'b0;
        end else if (push_btn) begin
            if (last_status == 1'b0) begin
                step <= 1'b1;
            end else begin
                step <= 1'b0;
            end
            last_status <= 1'b1;
        end else begin // push_btn == 0
            last_status <= 1'b0;
            step <= 1'b0;
        end
    end

endmodule
