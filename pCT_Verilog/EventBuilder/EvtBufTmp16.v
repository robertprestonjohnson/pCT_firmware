//Module to buffer 16 bit data words and at the end of the run send them
//out serially to the UART RAM buffer.  This is only used in case that
//the Ethernet connection is not available.
//R. Johnson & Ari Warren 7/29/2013
module EvtBufTmp16(DataStream,Send,DataIn,StrobeIn,Clock,Reset);
input Clock;
input Reset;
input StrobeIn;
input [15:0] DataIn;
input Send;
output DataStream;

reg DataStream;
reg [14:0] WriteAddress,ReadAddress;
wire [15:0] DataIn,DataOut,doa; 
reg ReadEnable;

EvtRamTmp EvtRamTmpI(Clock,Clock,StrobeIn,ReadEnable,StrobeIn,WriteAddress,ReadAddress,DataIn,doa,DataOut);

//initial begin
//	$display("Time              State StrobeIn    DataIn    Send DtStrm    Icnt WriteAddress ReadAddress ReadEnable    ShftOut");
//end 

//always @ (posedge Clock) begin
//	$display("%g\t EvtBufTmp %b    %b   %b   %b     %b       %b %b %b     %b      %b",$time,State,StrobeIn,DataIn,Send,DataStream,Icnt,WriteAddress,ReadAddress,ReadEnable,ShftOut);
//end

always @ (posedge Clock) begin
	if (Reset | RstAddr) begin
		WriteAddress <= 0;
	end else begin
		if (StrobeIn && (WriteAddress<15'd32767)) WriteAddress <= WriteAddress + 1;
	end
end

//State machine to empty the RAM serially into DataStream
parameter [4:0] Wait=5'b00001;	//Wait for the Send signal
parameter [4:0] Frst=5'b00010;	//Read the first word from memory
parameter [4:0] Serl=5'b00100;	//Send the first 11 bits out serially
parameter [4:0] Next=5'b01000;	//Send the 12th bit and read the next word from memory
parameter [4:0] Done=5'b10000;	//When done, reset the write pointer

reg [4:0] State, NextState;
reg [3:0] Icnt;
reg [15:0] ShftOut;
reg RstAddr;

always @ (State or Send or ShftOut or Icnt or ReadAddress or WriteAddress) begin
	case (State)
		Wait:	begin
					if (Send && (ReadAddress != WriteAddress)) begin
						NextState = Frst;
						ReadEnable = 1'b1;
					end else begin
						NextState = Wait;
						ReadEnable = 1'b0;
					end
					DataStream = 1'b0;
				end
		Frst:	begin
					ReadEnable = 1'b0;
					DataStream = 1'b0;
					NextState = Serl;
				end
		Serl:	begin
					DataStream = ShftOut[15];
					if (Icnt == 15) begin
						NextState = Next;
						ReadEnable = 1'b1;
					end else begin	
						NextState = Serl;
						ReadEnable = 1'b0;
					end
				end
		Next:	begin
					ReadEnable = 1'b0;
					DataStream = ShftOut[15];
					if (ReadAddress == WriteAddress-1) NextState = Done;
					else NextState = Serl;
				end
		Done:	begin
					NextState = Wait;
					ReadEnable = 1'b0;
					DataStream = 1'b0;
				end
		default: 	begin
						NextState = Wait;
						ReadEnable = 1'b0;
						DataStream = 1'b0;
					end
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin	
		State <= Wait;
	end else begin
		State <= NextState;
//		if (State != Wait) $display("%g\t EvtBufTmp: State=%b, ReadAddress=%d, WriteAddress=%d, Icnt=%d, ShftOut=%b, RstAddr=%b, StrobeIn=%b, ReadEnable=%b, DataStream=%b",$time,State,ReadAddress,WriteAddress,Icnt,ShftOut,RstAddr,StrobeIn,ReadEnable,DataStream);
		case (State)
			Wait:	begin
						ReadAddress <= 0;
						RstAddr <= 1'b0;
						Icnt <= 0;
					end
			Frst:	begin
						Icnt <= Icnt + 1;
						ShftOut <= DataOut;
						ReadAddress <= ReadAddress + 1;
					end
			Serl:	begin
						Icnt <= Icnt + 1;
						ShftOut <= {ShftOut[14:0],1'b0};
					end
			Next:	begin
						ShftOut <= DataOut;
						Icnt <= 1;
						if (ReadAddress != WriteAddress) ReadAddress <= ReadAddress + 1;
					end
			Done:	begin
						RstAddr <= 1'b1;
					end
		endcase
	end
end

endmodule

module EvtRamTmp (clka, clkb, ena, enb, wea, addra, addrb, dia, doa, dob);
input clka, clkb;
input wea;                  //Write enable
input ena, enb;             //Primary and secondary read enables
input [14:0] addra, addrb;  //Addresses for the primary and secondary ports
input [15:0] dia;           //Input data register to write to memory
output [15:0] doa, dob;     //Output registers for the two ports
reg [15:0] RAM [32767:0];    //Memory array
reg [15:0] dob, doa;

always @(posedge clka)
begin
	if (ena)
	begin
		if (wea) begin
			RAM[addra]<=dia;  
			doa <= dia;
			$display("%g\t EvtBufTmp: storing %b in address %d",$time,dia,addra);
		end else doa <= RAM[addra];        //Write first, then read
	end
end

always @(posedge clkb)
begin
	if (enb)
	begin
		dob <= RAM[addrb];
		$display("%g\t EvtBufTmp: reading %b from address %d",$time,RAM[addrb],addrb);
	end
end
endmodule
