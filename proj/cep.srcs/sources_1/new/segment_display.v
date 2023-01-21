`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/19/2023 03:27:12 PM
// Design Name: 
// Module Name: segment_display
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module segment_display(SW, CLK, SSEG_CA,SSEG_AN, LED);
input [3:0] SW;
input CLK;

output reg[7:0] SSEG_CA;
output reg[7:0] SSEG_AN;

output reg[3:0] LED;

slow_clock S1 (CLK , Clk_Slow);

initial begin
SSEG_AN <= 8'b11111110;
end
always @(posedge CLK)
begin
case(SW)
4'b0000: SSEG_CA <= 8'b11000000;
4'b0001: SSEG_CA <= 8'b11111001;
4'b0010: SSEG_CA <= 8'b10100100;
4'b0011: SSEG_CA <= 8'b10110000;
4'b0100: SSEG_CA <= 8'b10011001;
4'b0101: SSEG_CA <= 8'b10010010;
4'b0110: SSEG_CA <= 8'b10000010;
4'b0111: SSEG_CA <= 8'b11011000;
4'b1000: SSEG_CA <= 8'b10000000;
4'b1001: SSEG_CA <= 8'b10011000;
4'b1010: SSEG_CA <= 8'b10001000;
4'b1011: SSEG_CA <= 8'b10000000;
4'b1100: SSEG_CA <= 8'b11000110;
4'b1101: SSEG_CA <= 8'b10100001;
4'b1110: SSEG_CA <= 8'b10000110;
4'b1111: SSEG_CA <= 8'b10001110;
endcase
LED<=SW;

case(SSEG_AN)
8'b11111110: SSEG_AN <= 8'b11111101;
8'b11111101: SSEG_AN <= 8'b11111011;
8'b11111011: SSEG_AN <= 8'b11110111;
8'b11110111: SSEG_AN <= 8'b11101111;
8'b11101111: SSEG_AN <= 8'b11011111;
8'b11011111: SSEG_AN <= 8'b10111111;
8'b10111111: SSEG_AN <= 8'b01111111;
8'b01111111: SSEG_AN <= 8'b11111110;
endcase

 end
endmodule
 module slow_clock(CLK,Clk_Slow);
 input CLK;
 output Clk_Slow;
 reg[31:0] counter_out;
 reg Clk_Slow;
 initial 
 begin
 counter_out<=32'h00000000;
 Clk_Slow<=0;
 end
 always @(posedge CLK) begin
 counter_out<=counter_out + 32'h00000001;
 if (counter_out> 32'h00F5E100)
 begin
 counter_out<=32'h00000000;
 Clk_Slow<=!Clk_Slow;
 end
 end
 endmodule
 
module PWM_generator(CLK,DUTY_CYCLE, PWM_OUT);
input CLK;
output PWM_OUT;

reg[3:0] PWM_counter;
output reg[3:0] DUTY_CYCLE=5;

always @(posedge CLK) begin
    PWM_counter<=PWM_counter+1;
    if(PWM_counter>=9)
    PWM_counter<=0;
end
assign PWM_OUT = PWM_counter < DUTY_CYCLE? 1:0;
endmodule




 module rgb_to_pwm(RGBval, CLK, PWM_R, PWM_G, PWM_B);
    input [23:0] RGBval;
    input CLK;
    output PWM_R;
    output PWM_G;
    output PWM_B;


    PWM_generator PWM_Rgen (CLK,RGBval[23:16]>>1, PWM_R);
    PWM_generator PWM_Ggen (CLK,RGBval[15:8]>>1, PWM_G);
    PWM_generator PWM_Bgen (CLK, RGBval[7:0]>>1, PWM_B);

endmodule


    
    

 
//  module get_temp(SDA, SCL, CLK, temp);
//     inout SDA;
//     input SCL;
//     input CLK;
//     output reg[7:0] temp;
//     reg [7:0] count;
//     reg [2:0] mode;
//     const init_and_address = 9'b010010000;
// reg [8:0] shift_reg;
//  assign shift_out = shift_reg[8];
//  assign shift_in = shift_out
//     always @(posedge shift_clk) begin
//         shift_reg <= shift_reg << 1;
//         shift_reg[0] <= shift_in;
//     end

//     initial begin
//         assign SDA = 1;
//     end

//     always @(posedge CLK) begin
//         if(count==8'b11111111)begin
//         count <= 0;
//         end
//         else begin
//         count <= count + 1;
//         end
//     end



    



    