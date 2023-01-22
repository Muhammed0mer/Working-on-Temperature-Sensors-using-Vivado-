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

module thermometer(CLK, SCL, SDA, SSEG_CA, SSEG_AN, PWM_R, PWM_G, PWM_B);
input CLK;
input SCL;
inout SDA;
output reg [7:0] SSEG_CA;
output reg [7:0] SSEG_AN;
output reg [7:0] PWM_R;
output reg [7:0] PWM_G;
output reg [7:0] PWM_B;

reg [7:0] temp;
reg temp_sign;
wire [23:0] RGBval;
wire [3:0] sw;

initial begin
SSEG_AN <= 8'b11111110;
sw <= 4'b1010;  // 4'b1010 is blank
end


temp_7seg_encoder encd(.CLK(CLK), .SSEG_AN(SSEG_AN), .temp(temp), .temp_sign(temp_sign), .sw(sw));
segment_display disp(.SW(sw), .CLK(CLK), .SSEG_CA(SSEG_CA), .SSEG_AN(SSEG_AN));

temp_RGB_encoder RGBencd(.CLK(CLK), .temp(temp), .temp_sign(temp_sign), .RGBval(RGBval));
RGB_PWM RGB(.CLK(CLK), .RGBval(RGBval), .PWM_R(PWM_R), .PWM_G(PWM_G), .PWM_B(PWM_B));

endmodule


module temp_7seg_encoder(CLK,SSEG_AN, temp,temp_sign, sw);
input CLK;
input [7:0] temp;
input temp_sign;
input [7:0] SSEG_AN;
output reg [3:0] sw;

reg [11:0] temp_BCD;

Binary_to_BCD #(.INPUT_WIDTH(8), .DECIMAL_DIGITS(3)) temp_BCD_encoder(.i_Clock(CLK), .i_Binary(temp), .i_Start(1'b1), .o_BCD(temp_BCD), .o_DV());

always @(SSEG_AN) begin
case(SSEG_AN)
8'b11111110: sw <= temp_BCD[3:0];
8'b11111101: sw <= temp_BCD[7:4];
8'b11111011: sw <= temp_BCD[11:8];
8'b11110111: if(temp_sign == 1'b1) sw <= 4'b1111;
default: sw <= 4'b1010;
endcase
end

endmodule

module temp_RGB_encoder(CLK, temp,temp_sign, RGBval);
input CLK;
input [7:0] temp;
input temp_sign;
output reg [23:0] RGBval;



always @(posedge CLK) begin
if(temp < 8'd32 || temp_sign) begin
RGBval <= {8'd66,8'd245,8'd66+(temp<<2)};
end
else if(temp > 8'd32 && temp < 8'd38) begin
RGBval <= {8'd255,8'd255-(temp<<2),8'd66};
end
else begin
RGBval <= {8'd255,8'd0,8'd0}; 
end
end

endmodule


module segment_display(SW, CLK, SSEG_CA,SSEG_AN);
input [3:0] SW;
input CLK;

output reg[7:0] SSEG_CA;
output reg[7:0] SSEG_AN;

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
4'b1010: SSEG_CA <= 8'b11111111;    // A replaced with blank
4'b1011: SSEG_CA <= 8'b10000000;
4'b1100: SSEG_CA <= 8'b11000110;
4'b1101: SSEG_CA <= 8'b10100001;
4'b1110: SSEG_CA <= 8'b10000110;
4'b1111: SSEG_CA <= 8'b10111111;   // F replaced with minus sign
endcase


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
 
 module rgb_to_pwm(RGBval, CLK, PWM_R, PWM_G, PWM_B);
    input [23:0] RGBval;
    input CLK;
    output PWM_R;
    output PWM_G;
    output PWM_B;


    PWM_generator PWM_Rgen (CLK,RGBval[23:16], PWM_R);
    PWM_generator PWM_Ggen (CLK,RGBval[15:8], PWM_G);
    PWM_generator PWM_Bgen (CLK, RGBval[7:0], PWM_B);

endmodule

module PWM_generator(CLK,DUTY_CYCLE, PWM_OUT);
input CLK;
input reg[7:0] DUTY_CYCLE;
output PWM_OUT;

reg[7:0] PWM_counter=0;


always @(posedge CLK) begin
    PWM_counter<=PWM_counter+1;
    if(PWM_counter>=255)
    PWM_counter<=0;
end
assign PWM_OUT = PWM_counter < DUTY_CYCLE? 1:0;
endmodule

module Binary_to_BCD
#(parameter INPUT_WIDTH,
  parameter DECIMAL_DIGITS)
(
 input                         i_Clock,
 input [INPUT_WIDTH-1:0]       i_Binary,
 input                         i_Start,
 //
 output [DECIMAL_DIGITS*4-1:0] o_BCD,
 output                        o_DV
 );
 
parameter s_IDLE              = 3'b000;
parameter s_SHIFT             = 3'b001;
parameter s_CHECK_SHIFT_INDEX = 3'b010;
parameter s_ADD               = 3'b011;
parameter s_CHECK_DIGIT_INDEX = 3'b100;
parameter s_BCD_DONE          = 3'b101;
 
reg [2:0] r_SM_Main = s_IDLE;
 
// The vector that contains the output BCD
reg [DECIMAL_DIGITS*4-1:0] r_BCD = 0;
  
// The vector that contains the input binary value being shifted.
reg [INPUT_WIDTH-1:0]      r_Binary = 0;
    
// Keeps track of which Decimal Digit we are indexing
reg [DECIMAL_DIGITS-1:0]   r_Digit_Index = 0;
  
// Keeps track of which loop iteration we are on.
// Number of loops performed = INPUT_WIDTH
reg [7:0]                  r_Loop_Count = 0;

wire [3:0]                 w_BCD_Digit;
reg                        r_DV = 1'b0;                       
  
always @(posedge i_Clock)
  begin

    case (r_SM_Main) 

      // Stay in this state until i_Start comes along
      s_IDLE :
        begin
          r_DV <= 1'b0;
           
          if (i_Start == 1'b1)
            begin
              r_Binary  <= i_Binary;
              r_SM_Main <= s_SHIFT;
              r_BCD     <= 0;
            end
          else
            r_SM_Main <= s_IDLE;
        end
               

      // Always shift the BCD Vector until we have shifted all bits through
      // Shift the most significant bit of r_Binary into r_BCD lowest bit.
      s_SHIFT :
        begin
          r_BCD     <= r_BCD << 1;
          r_BCD[0]  <= r_Binary[INPUT_WIDTH-1];
          r_Binary  <= r_Binary << 1;
          r_SM_Main <= s_CHECK_SHIFT_INDEX;
        end          
       

      // Check if we are done with shifting in r_Binary vector
      s_CHECK_SHIFT_INDEX :
        begin
          if (r_Loop_Count == INPUT_WIDTH-1)
            begin
              r_Loop_Count <= 0;
              r_SM_Main    <= s_BCD_DONE;
            end
          else
            begin
              r_Loop_Count <= r_Loop_Count + 1;
              r_SM_Main    <= s_ADD; end end // Break down each BCD Digit individually. Check them one-by-one to // see if they are greater than 4. If they are, increment by 3. // Put the result back into r_BCD Vector. 
              s_ADD : begin if (w_BCD_Digit > 4)
            begin                                     
              r_BCD[(r_Digit_Index*4)+:4] <= w_BCD_Digit + 3;  
            end
           
          r_SM_Main <= s_CHECK_DIGIT_INDEX; 
        end       
       
       
      // Check if we are done incrementing all of the BCD Digits
      s_CHECK_DIGIT_INDEX :
        begin
          if (r_Digit_Index == DECIMAL_DIGITS-1)
            begin
              r_Digit_Index <= 0;
              r_SM_Main     <= s_SHIFT;
            end
          else
            begin
              r_Digit_Index <= r_Digit_Index + 1;
              r_SM_Main     <= s_ADD;
            end
        end
       


      s_BCD_DONE :
        begin
          r_DV      <= 1'b1;
          r_SM_Main <= s_IDLE;
        end
       
       
      default :   
      r_SM_Main <= s_IDLE;
    

    endcase
  end // always @ (posedge i_Clock)  

 
assign w_BCD_Digit = r_BCD[r_Digit_Index*4 +: 4];
     
assign o_BCD = r_BCD;
assign o_DV  = r_DV;
    
endmodule // Binary_to_BCD
    
    

 
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



    



    