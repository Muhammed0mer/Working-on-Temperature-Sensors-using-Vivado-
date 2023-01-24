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

module thermometer(CLK,reset, TMP_SCL, TMP_SDA, SSEG_CA, SSEG_AN, PWM_R, PWM_G, PWM_B);
input         CLK;        // nexys clk signal
    input         reset;         // btnC on nexys
    inout         TMP_SDA;          // i2c sda on temp sensor - bidirectional
    output        TMP_SCL ;         // i2c scl on temp sensor
//input CLK;
//output SCL;
//inout SDA;
output [7:0] SSEG_CA;
output [7:0] SSEG_AN;
output PWM_R;
output PWM_G;
output PWM_B;

wire [15:0] raw_temp_wire;
reg [15:0] raw_temp;
reg [7:0] temp;
reg temp_sign;
wire [23:0] RGBval;
wire [3:0] sw;

 wire sda_dir;                   // direction of SDA signal - to or from master
    wire w_200kHz;                  // 200kHz SCL
    wire [15:0] w_data;              // 8 bits of temperature data

    // Instantiate i2c master
    i2c_master master(
        .clk_200kHz(w_200kHz),
        .reset(reset),
        .temp_data(w_data),
        .SDA(TMP_SDA),
        .SDA_dir(sda_dir),
        .SCL(TMP_SCL)
    );
    
    // Instantiate 200kHz clock generator
    clkgen_200kHz cgen(
        .clk_100MHz(CLK),
        .clk_200kHz(w_200kHz)
    );
//initial begin
//SSEG_AN <= 8'b11111110;
////sw <= 4'b1010;  // 4'b1010 is blank
//end

//i2c_temp adt_interface(SDA,CLK,raw_temp_wire);
always @ (posedge CLK) begin
temp_sign <= w_data[15];
temp <= w_data[14:7];
if (temp_sign) temp <= w_data[14:7] - 256;
end

temp_7seg_encoder encd(.CLK(CLK), .SSEG_AN(SSEG_AN), .temp(temp), .temp_sign(temp_sign), .sw(sw));
segment_display disp(.SW(sw), .CLK(CLK), .SSEG_CA(SSEG_CA), .SSEG_AN(SSEG_AN));

temp_RGB_encoder RGBencd(.CLK(CLK), .temp(temp), .temp_sign(temp_sign), .RGBval(RGBval));
rgb_to_pwm RGB(.CLK(CLK), .RGBval(RGBval), .PWM_R(PWM_R), .PWM_G(PWM_G), .PWM_B(PWM_B));



endmodule


module temp_7seg_encoder(CLK,SSEG_AN, temp,temp_sign, sw);
input CLK;
input [7:0] temp;
input temp_sign;
input [7:0] SSEG_AN;
output reg [3:0] sw;

wire [11:0] temp_BCD_wire;
reg [11:0] temp_BCD;

Binary_to_BCD #(.INPUT_WIDTH(8), .DECIMAL_DIGITS(3)) temp_BCD_encoder(.i_Clock(CLK), .i_Binary(temp), .i_Start(1'b1), .o_BCD(temp_BCD_wire), .o_DV());

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
input[7:0] DUTY_CYCLE;
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
#(parameter INPUT_WIDTH=8,
  parameter DECIMAL_DIGITS=3)
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
    
    

 
// module get_temp(SDA, SCL, CLK, rawtemp);
//    inout SDA;
//    inout SCL;
//    input CLK;
//    output reg[7:0] rawtemp;
//    reg [7:0] count=0;
//    reg [2:0] mode;
//    const init_and_address = 10'b1010010000;

//    reg [9:0] shift_reg = init_and_address;
//    assign shift_out = shift_reg[8];
//    assign shift_in = shift_out;
//    always @(posedge SCL) begin
//        if(mode==3'b000)begin
//        SDA <= shift_out;
//        shift_reg <= shift_reg << 1;
//        shift_reg[0] <= shift_in;
//        count <= count + 1;
//        if(count==10) begin
//        mode <= 3'b001;
//        count <= 0;
//        end
//        end
//        else if(mode==3'b001)begin
//            if (SDA==0) mode <= 3'b010;
//            else mode <= 3'b000;
//           end
//        else if (mode==3'b010) begin
//            if (count<=8)begin
//            SDA <= 1;
//            count <= count + 1;
//            end
//            else if(count==9 && SDA==0) mode <= 3'b011;
//            else mode <= 3'b000;
//            end
//        else if (mode==3'b011)begin
//            if (count<=10)begin
//            SDA <= shift_out;
//        shift_reg <= shift_reg << 1;
//        shift_reg[0] <= shift_in;
//        count <= count + 1;
//        end
//        else if (count ==11 && SDA==0) begin
//        count <= count+1;
//        end
//        else if (count>11 && count<=27) begin
//            if (count==20) SDA<=1;
//            else rawtemp <= {rawtemp<<1,SDA};
//            count <= count + 1;
//        end
//        else 
//            mode <= 3'b000;
//        end


            





//    initial begin
//        assign SDA = 1;
//    end

//    always @(posedge CLK) begin
//        if(count==8'b11111111)begin
//        count <= 0;
//        end
//        else begin
//        count <= count + 1;
//        end
//    end
//        endmodule

//module i2c_temp(
//input SCL,       // I2C clock input
//inout SDA,       // I2C data input
//output reg [15:0] rawtemp   // 8-bit temperature register to store the temperature value
//);
//// I2C address for the temperature sensor
//reg SDAreg;
//assign SDA = SDAreg;
//parameter DEVICEADDRESS = 9'b101001000;
//parameter REGADDRESS=8'h00;
//reg [9:0] address=DEVICEADDRESS;    // 8-bit address register to store the sensor's address
//reg [7:0] reg_address = REGADDRESS;     // 16-bit data register to store the data from the sensor
//reg [4:0] bit_count=0;  // 4-bit bit counter to keep track of the current bit
//reg [2:0] state=0;      // 2-bit state machine to keep track of the current state
//reg ack;              // Acknowledge signal
//reg[4:0] count=0;


//// State machine states
//localparam S_START = 3'b000,  S_WAIT_RESTART_ACK=3'b001, S_ADDRESS = 3'b010,S_DATA = 3'b011, S_RESTART = 3'b100, S_WAIT_ACK=3'b101, S_REG_ADDRESS=3'b110, S_WAIT_START_ACK=3'b111;

//always @(posedge SCL) begin
//case (state)
//S_START: begin
//    // Send start Scondition
//    //SDA <= 1'b0;
//    // Send address
//    bit_count<=0;
//        if (bit_count == 10) begin
//            // Send the address of the sensor
//            SDAreg<= 0;
//            // Move to data state
//            state <= S_WAIT_START_ACK;
//        end 
//        else begin
//            SDAreg <= address[bit_count];
//            // Increment bit count
//            bit_count <= bit_count + 1;
//        end
//    end
//    // Move to address state
//    //state <= S_ADDRESS;
////S_PAUSE: begin
////    //delay
////    if (count<5'b11111) count<=count+1;
////    else begin
////    // Move to address state
////    state <= S_DATA;
////end
////end
//S_WAIT_START_ACK: begin
//SDAreg<=1;
//if(SDAreg==0) begin
//state<=S_REG_ADDRESS;
//end
//end
//S_WAIT_ACK: begin
//SDAreg<=1;
//if(SDAreg==0) begin
//state<=S_RESTART;
//end
//end
//S_WAIT_RESTART_ACK: begin
//SDAreg<=1;
//if(SDA==0) begin
//state<=S_DATA;
//end
//end
//S_ADDRESS: begin
//    // Send address
//    if (bit_count == 8) begin
//        // Send the address of the sensor
//        SDAreg <= address[bit_count];
//        // Move to data state
//        state <= S_WAIT_ACK;
//    end 
//    else begin
//        SDAreg <= address[bit_count];
//        // Increment bit count
//        bit_count <= bit_count + 1;
//    end
//end
//S_REG_ADDRESS: begin
//    bit_count<=0;
//// Send address
//    if (bit_count == 8) begin
//        // Send the address of the sensor
//        SDAreg <= reg_address[bit_count];
//        // Move to data state
//        state <= S_WAIT_ACK;
//    end 
//    else begin
//        SDAreg <= reg_address[bit_count];
//        // Increment bit count
//        bit_count <= bit_count + 1;
//    end
//end
//S_RESTART:begin
//// Send address
//        if (bit_count == 10) begin
//            // Send the address of the sensor
//            SDAreg <= 1;
//            // Move to data state
//            state <= S_WAIT_RESTART_ACK;
//        end 
//        else begin
//            SDAreg <= address[bit_count];
//            // Increment bit count
//            bit_count <= bit_count + 1;
//        end
//    end
//S_DATA: begin
//    bit_count<=0;
//    if(bit_count==8) begin
//        // Send the data to the sensor
//        SDAreg <= 0;
//        // Move to stop state
//    end 
//    else if (bit_count==16) begin
//    SDAreg<=1;
//    state<= S_RESTART;
//    end
//    else begin
//        rawtemp<={rawtemp<<1,SDAreg};
//        // Increment bit count
//        bit_count <= bit_count + 1;
//    end
//    end
//    endcase
//end

//endmodule
    
module clkgen_200kHz(
    input clk_100MHz,
    output clk_200kHz
    );
    
    // 100 x 10^6 / 200 x 10^3 / 2 = 250 <-- 8 bit counter
    reg [7:0] counter = 8'h00;
    reg clk_reg = 1'b1;
    
    always @(posedge clk_100MHz) begin
        if(counter == 249) begin
            counter <= 8'h00;
            clk_reg <= ~clk_reg;
        end
        else
            counter <= counter + 1;
    end
    
    assign clk_200kHz = clk_reg;
    
endmodule

module i2c_master(
    input clk_200kHz,               // i_clk
    input reset,                    // btnC on nexys
    inout SDA,                      // i2c standard interface signal
    output [15:0] temp_data,         // 8 bits binary representation of deg C
    output SDA_dir,                 // direction of inout signal on SDA - to/from master 
    output SCL                      // i2c standard interface signal - 10KHZ
    );
    
    // *** GENERATE 10kHz SCL clock from 200kHz ***************************
    // 200 x 10^3 / 10 x 10^3 / 2 = 10
    reg [3:0] counter = 4'b0;  // count up to 9
    reg clk_reg = 1'b1; 
    
    always @(posedge clk_200kHz or posedge reset) begin
        if(reset) begin
            counter = 4'b0;
            clk_reg = 1'b0;
        end
        else 
            if(counter == 9) begin
                counter <= 4'b0;
                clk_reg <= ~clk_reg;    // toggle reg
            end
            else
                counter <= counter + 1;
      end
    // Set value of i2c SCL signal to the sensor - 10kHz            
    assign SCL = clk_reg;   
    // ********************************************************************     

    // Signal Declarations               
    parameter [7:0] sensor_address_plus_read = 8'b1001_0111;// 0x97
    reg [7:0] tMSB = 8'b0;                                  // Temp data MSB
    reg [7:0] tLSB = 8'b0;                                  // Temp data LSB
    reg o_bit = 1'b1;                                       // output bit to SDA - starts HIGH
    reg [11:0] count = 12'b0;                               // State Machine Synchronizing Counter
    reg [15:0] temp_data_reg;					            // Temp data buffer register			

    // State Declarations - need 28 states
    localparam [4:0] POWER_UP   = 5'h00,
                     START      = 5'h01,
                     SEND_ADDR6 = 5'h02,
					 SEND_ADDR5 = 5'h03,
					 SEND_ADDR4 = 5'h04,
					 SEND_ADDR3 = 5'h05,
					 SEND_ADDR2 = 5'h06,
					 SEND_ADDR1 = 5'h07,
					 SEND_ADDR0 = 5'h08,
					 SEND_RW    = 5'h09,
                     REC_ACK    = 5'h0A,
                     REC_MSB7   = 5'h0B,
					 REC_MSB6	= 5'h0C,
					 REC_MSB5	= 5'h0D,
					 REC_MSB4	= 5'h0E,
					 REC_MSB3	= 5'h0F,
					 REC_MSB2	= 5'h10,
					 REC_MSB1	= 5'h11,
					 REC_MSB0	= 5'h12,
                     SEND_ACK   = 5'h13,
                     REC_LSB7   = 5'h14,
					 REC_LSB6	= 5'h15,
					 REC_LSB5	= 5'h16,
					 REC_LSB4	= 5'h17,
					 REC_LSB3	= 5'h18,
					 REC_LSB2	= 5'h19,
					 REC_LSB1	= 5'h1A,
					 REC_LSB0	= 5'h1B,
                     NACK       = 5'h1C;
      
    reg [4:0] state_reg = POWER_UP;                         // state register
                       
    always @(posedge clk_200kHz or posedge reset) begin
        if(reset) begin
            state_reg <= START;
			count <= 12'd2000;
        end
        else begin
			count <= count + 1;
            case(state_reg)
                POWER_UP    : begin
                                if(count == 12'd1999)
                                    state_reg <= START;
                end
                START       : begin
                                if(count == 12'd2004)
                                    o_bit <= 1'b0;          // send START condition 1/4 clock after SCL goes high    
                                if(count == 12'd2013)
                                    state_reg <= SEND_ADDR6; 
                end
                SEND_ADDR6  : begin
                                o_bit <= sensor_address_plus_read[7];
                                if(count == 12'd2033)
                                    state_reg <= SEND_ADDR5;
                end
				SEND_ADDR5  : begin
                                o_bit <= sensor_address_plus_read[6];
                                if(count == 12'd2053)
                                    state_reg <= SEND_ADDR4;
                end
				SEND_ADDR4  : begin
                                o_bit <= sensor_address_plus_read[5];
                                if(count == 12'd2073)
                                    state_reg <= SEND_ADDR3;
                end
				SEND_ADDR3  : begin
                                o_bit <= sensor_address_plus_read[4];
                                if(count == 12'd2093)
                                    state_reg <= SEND_ADDR2;
                end
				SEND_ADDR2  : begin
                                o_bit <= sensor_address_plus_read[3];
                                if(count == 12'd2113)
                                    state_reg <= SEND_ADDR1;
                end
				SEND_ADDR1  : begin
                                o_bit <= sensor_address_plus_read[2];
                                if(count == 12'd2133)
                                    state_reg <= SEND_ADDR0;
                end
				SEND_ADDR0  : begin
                                o_bit <= sensor_address_plus_read[1];
                                if(count == 12'd2153)
                                    state_reg <= SEND_RW;
                end
				SEND_RW     : begin
                                o_bit <= sensor_address_plus_read[0];
				if(count == 12'd2169)
                                    state_reg <= REC_ACK;
                end
                REC_ACK     : begin
                                if(count == 12'd2189)
                                    state_reg <= REC_MSB7;
                end
                REC_MSB7     : begin
                                tMSB[7] <= i_bit;
                                if(count == 12'd2209)
                                    state_reg <= REC_MSB6;
                                
                end
				REC_MSB6     : begin
                                tMSB[6] <= i_bit;
                                if(count == 12'd2229)
                                    state_reg <= REC_MSB5;
                                
                end
				REC_MSB5     : begin
                                tMSB[5] <= i_bit;
                                if(count == 12'd2249)
                                    state_reg <= REC_MSB4;
                                
                end
				REC_MSB4     : begin
                                tMSB[4] <= i_bit;
                                if(count == 12'd2269)
                                    state_reg <= REC_MSB3;
                                
                end
				REC_MSB3     : begin
                                tMSB[3] <= i_bit;
                                if(count == 12'd2289)
                                    state_reg <= REC_MSB2;
                                
                end
				REC_MSB2     : begin
                                tMSB[2] <= i_bit;
                                if(count == 12'd2309)
                                    state_reg <= REC_MSB1;
                                
                end
				REC_MSB1     : begin
                                tMSB[1] <= i_bit;
                                if(count == 12'd2329)
                                    state_reg <= REC_MSB0;
                                
                end
				REC_MSB0     : begin
								o_bit <= 1'b0;
                                tMSB[0] <= i_bit;
                                if(count == 12'd2349)
                                    state_reg <= SEND_ACK;
                                
                end
                SEND_ACK   : begin
                                if(count == 12'd2369)
                                    state_reg <= REC_LSB7;
                end
                REC_LSB7    : begin
                                tLSB[7] <= i_bit;
                                if(count == 12'd2389)
									state_reg <= REC_LSB6;
                end
                REC_LSB6    : begin
                                tLSB[6] <= i_bit;
                                if(count == 12'd2409)
									state_reg <= REC_LSB5;
                end
				REC_LSB5    : begin
                                tLSB[5] <= i_bit;
                                if(count == 12'd2429)
									state_reg <= REC_LSB4;
                end
				REC_LSB4    : begin
                                tLSB[4] <= i_bit;
                                if(count == 12'd2449)
									state_reg <= REC_LSB3;
                end
				REC_LSB3    : begin
                                tLSB[3] <= i_bit;
                                if(count == 12'd2469)
									state_reg <= REC_LSB2;
                end
				REC_LSB2    : begin
                                tLSB[2] <= i_bit;
                                if(count == 12'd2489)
									state_reg <= REC_LSB1;
                end
				REC_LSB1    : begin
                                tLSB[1] <= i_bit;
                                if(count == 12'd2509)
									state_reg <= REC_LSB0;
                end
				REC_LSB0    : begin
								o_bit <= 1'b1;
                                tLSB[0] <= i_bit;
                                if(count == 12'd2529)
									state_reg <= NACK;
                end
                NACK        : begin
                                if(count == 12'd2559) begin
									count <= 12'd2000;
                                    state_reg <= START;
								end
                end
            endcase     
        end
    end       
    
    // Buffer for temperature data
    always @(posedge clk_200kHz)
        if(state_reg == NACK)
            temp_data_reg <= { tMSB[7:0], tLSB[7:0] };
    
    
    // Control direction of SDA bidirectional inout signal
    assign SDA_dir = (state_reg == POWER_UP || state_reg == START || state_reg == SEND_ADDR6 || state_reg == SEND_ADDR5 ||
					  state_reg == SEND_ADDR4 || state_reg == SEND_ADDR3 || state_reg == SEND_ADDR2 || state_reg == SEND_ADDR1 ||
                      state_reg == SEND_ADDR0 || state_reg == SEND_RW || state_reg == SEND_ACK || state_reg == NACK) ? 1 : 0;
    // Set the value of SDA for output - from master to sensor
    assign SDA = SDA_dir ? o_bit : 1'bz;
    // Set value of input wire when SDA is used as an input - from sensor to master
    assign i_bit = SDA;
    // Outputted temperature data
    assign temp_data = temp_data_reg;
 
endmodule



    