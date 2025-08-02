`define CNT       104
`define CNT_HALF  52
`define IDLE      2'b00
`define START     2'b01
`define DATA      2'b10
`define STOP      2'b11

module UART_RX (
    input wire clock,
    input wire reset,
    input wire rx,
    output srclk,
    output rclk,
    output ser
);

// baudrate 9600 counter
reg [13:0] counter;
reg syncff0, syncff1;
reg rx_;
reg [1:0] state, next;
reg [7:0] rdat,value;
reg [2:0] data_cnt;

wire counter_end, start, start_end, data_end, stop_end, get_dat;

assign get_dat      = (counter == `CNT_HALF); // giữa bit
assign counter_end  = (counter == `CNT);      // kết thúc 1 bit
assign start_end    = (state == `START) && counter_end;
assign data_end     = (state == `DATA)  && counter_end && (data_cnt == 3'd7);
assign stop_end     = (state == `STOP)  && counter_end;

// Đồng bộ tín hiệu rx
always @(posedge clock or negedge reset) begin
    if (!reset) begin
        syncff0 <= 1'b1;
        syncff1 <= 1'b1;
    end else begin
        syncff0 <= rx;
        syncff1 <= syncff0;
    end
end

// Phát hiện cạnh xuống cho start bit
always @(posedge clock or negedge reset) begin
    if (!reset) rx_ <= 1'b1;
    else        rx_ <= syncff1;
end

assign start = ~syncff1 & rx_;  // cạnh xuống (falling edge)

// State machine
always @(posedge clock or negedge reset) begin
    if (!reset) state <= `IDLE;
    else        state <= next;
end

always @(*) begin
    case (state)
        `IDLE:  next = start      ? `START : `IDLE;
        `START: next = start_end  ? `DATA  : `START;
        `DATA:  next = data_end   ? `STOP  : `DATA;
        `STOP:  next = stop_end   ? `IDLE  : `STOP;
        default:next = `IDLE;
    endcase
end

// Bộ đếm baudrate
always @(posedge clock or negedge reset) begin
    if (!reset)
        counter <= 0;
    else if (state != `IDLE) begin
        if (counter_end)
            counter <= 0;
        else
            counter <= counter + 1;
    end
end

// Đếm bit data
always @(posedge clock or negedge reset) begin
    if (!reset)
        data_cnt <= 0;
    else if ((state == `DATA) && counter_end)
        data_cnt <= data_cnt + 1;
end

// Ghi dữ liệu khi đến thời điểm lấy mẫu giữa bit
always @(posedge clock or negedge reset) begin
    if (!reset)
        rdat <= 8'b0;
    else if ((state == `DATA) && get_dat)
        rdat <= {syncff1, rdat[7:1]};  // shift từ LSB
end

// Gán dữ liệu ra LED sau khi kết thúc STOP
always @(posedge clock or negedge reset) begin
    if (!reset)
        value <= 8'b0;
    else if (stop_end)
        value <= rdat;
end


// chỉ gán dữ liệu khi stop_end
reg data_valid;

always @(posedge clock or negedge reset) begin
    if (!reset) begin
        data_valid <= 0;
    end else begin
        data_valid <= stop_end;
    end
end

    led_driver_4digit_595 led_display (
        .clk(clock),
        .reset(reset),
        .value(value ),
		  .data_valid(data_valid),
        .srclk(srclk),
        .rclk(rclk),
        .ser(ser)
    );



endmodule
module led_driver_4digit_595(
    input wire clk,
    input wire reset,
    input wire [7:0] value,
	 input data_valid,
    output reg srclk,
    output reg rclk,
    output reg ser
);

    reg [15:0] shift_data;
    reg [4:0] bit_cnt;
    reg [2:0] state;
    reg [7:0] seg_code;
    reg [7:0] digit_sel;
    reg [3:0] digit_val;
    reg [1:0] digit_index;
    reg [7:0] buffer[3:0];
    reg [1:0] count;
    reg enter_received;

    reg [3:0] digits[3:0];
    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            count <= 0;
            enter_received <= 0;
            buffer[0] <= 0;
            buffer[1] <= 0;
            buffer[2] <= 0;
            buffer[3] <= 0;
            digits[0] <= 0;
            digits[1] <= 0;
            digits[2] <= 0;
            digits[3] <= 0;
        end else begin
            if (enter_received) begin
                // Cập nhật giá trị hiển thị từ buffer
                digits[0] <= buffer[3];
                digits[1] <= buffer[2];
                digits[2] <= buffer[1];
                digits[3] <= buffer[0];


                count <= 0;
                enter_received <= 0;
            end else
			if (data_valid) begin	
				if (value == 8'd13) begin
                // Nhận phím Enter
                enter_received <= 1;
            end else if (value >= 8'd48 && value <= 8'd57) begin
                // Nhận số từ '0' đến '9'
                if (count < 4) begin
                    buffer[count] <= value - 8'd48;
                    count <= count + 1;
                end
            end
			end
        end
    end

    function [7:0] seg7;
        input [3:0] d;
        begin
            case (d)
                4'd0: seg7 = 8'b00111111;
                4'd1: seg7 = 8'b00000110;
                4'd2: seg7 = 8'b01011011;
                4'd3: seg7 = 8'b01001111;
                4'd4: seg7 = 8'b01100110;
                4'd5: seg7 = 8'b01101101;
                4'd6: seg7 = 8'b01111101;
                4'd7: seg7 = 8'b00000111;
                4'd8: seg7 = 8'b01111111;
                4'd9: seg7 = 8'b01101111;
                default: seg7 = 8'b00000000;
            endcase
        end
    endfunction

    localparam IDLE      = 3'd0,
               SHIFT_HI  = 3'd1,
               SHIFT_LO  = 3'd2,
               LATCH_HI  = 3'd3,
               LATCH_LO  = 3'd4;

    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            srclk <= 0;
            rclk <= 0;
            ser <= 0;
            bit_cnt <= 0;
            digit_index <= 0;
            state <= IDLE;
        end else begin
            case (state)
                IDLE: begin
                    digit_val <= digits[digit_index];
                    case (digit_index)
                        2'd0: digit_sel <= 8'b11110111;
                        2'd1: digit_sel <= 8'b11111011;
                        2'd2: digit_sel <= 8'b11111101;
                        2'd3: digit_sel <= 8'b11111110;
                        default: digit_sel <= 8'b11111111;
                    endcase
                    seg_code <= seg7(digit_val);
                    shift_data <= {~seg_code, ~digit_sel};
                    bit_cnt <= 15;
                    state <= SHIFT_HI;
                end
                SHIFT_HI: begin
                    ser <= shift_data[bit_cnt];
                    srclk <= 1;
                    state <= SHIFT_LO;
                end
                SHIFT_LO: begin
                    srclk <= 0;
                    if (bit_cnt == 0)
                        state <= LATCH_HI;
                    else begin
                        bit_cnt <= bit_cnt - 1;
                        state <= SHIFT_HI;
                    end
                end
                LATCH_HI: begin
                    rclk <= 1;
                    state <= LATCH_LO;
                end
                LATCH_LO: begin
                    rclk <= 0;
                    digit_index <= (digit_index == 3) ? 0 : digit_index + 1;
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule

