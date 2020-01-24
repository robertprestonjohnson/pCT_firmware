//This program monitors the data output from a pCTFE64 test board FPGA and writes the data into a 65kByte RAM
//buffer (block RAM) whenever it encouters a data packet indicated by a start bit.  One state machine
//monitors the data stream to calculate when the data packet starts and finishes to turn on and off
//the writing to RAM.  Another state machine writes a byte to RAM every time it accumulates 8 bits.
//A third state machine can be activated to dump the data out of the RAM one byte at a time, intended
//to go to a UART for slow transmission to the PC.  The first two bytes sent out give the number of
//data bytes to follow, encoded as a 16-bit integer, MSB sent first.  Once the RAM is read out it is
//reset such that further data to come will overwrite the old data.
//R. Johnson  10/22/2012
//This version does not try to parse the incoming data stream.  It simply looks for a start bit to begin
//filling the RAM, and it turns off the RAM filling if a long string of zeroes is encountered.
module DataRAMbufferSimple(TxD_start, TxD_data, DMPDone, DMPStart, TxD_busy, SerialDataIn, Clock, Reset, Instance);
input [3:0] Instance;     	//Instance number, for debugging
input Reset;			 	//System initialization reset 
input Clock;			 	//Fast system clock
output TxD_start;        	//Signal start of UART output
output [7:0] TxD_data;   	//UART output byte
input TxD_busy;          	//UART transmitter busy signal
input SerialDataIn;      	//Serial data stream from the pCTFE64 chip
input DMPStart;          	//Signal to start dumping the RAM buffer to the UART
output DMPDone;          	//Signal that the RAM dump is complete

reg DMPDone, TxD_start;
reg [7:0] TxD_data;

reg [89:0] SerialDataDly;    //Delay the data stream to allow look-ahead
wire SerialData;

//State machine to monitor the test board's output stream to turn on and off the dumping of the data to block RAM
parameter [2:0] QM0= 3'b000;   
parameter [2:0] QM1= 3'b001;   
parameter [2:0] QM2= 3'b010;   
parameter [2:0] QM3= 3'b011;   
parameter [2:0] QM4= 3'b100;   
reg [2:0] StateMon, NextStateMon;
reg [6:0] Cntm;

//initial begin
//	$display("Time SerialData StateMon StateRAM StateDMP TxDstart TxDdata RMSt RMSp Wadr Radr RAMbyte doa dob wea ena enb CntByte Ncl CntCl Header CntCh Cnth Cntm DatIn ChpHdr SDatDly");
//end

//always @ (posedge Clock) begin
//   $display("%g\t    %b       %h      %h        %h        %b     %b   %b  %b %h %h %b %b %b %b %b %b   %h   %h   %h  %b  %h  %h  %h %b %b %b",$time,SerialData,StateMon,StateRAM,StateDMP,TxD_start,TxD_data,RAMStart,RAMStop,WriteAdr,ReadAdr,RAMbyte,doa,dob,wea,ena,enb,CntByte,Nclus,CntCl,Header,CntCh,Cnth,Cntm,SerialDataIn,ChipHdr,SerialDataDly);
//end

//Use a shift register to delay the data stream so that the code can look ahead for start bits
always @ (posedge Clock) begin
	SerialDataDly <= {SerialDataDly[88:0],SerialDataIn};
end
assign SerialData = SerialDataDly[65];

always @ (StateMon or Cntm or SerialData or SerialDataDly or StateRAM or DMPStart)
begin
	case (StateMon)
		QM0: begin
				if (SerialDataDly == 0) NextStateMon = QM1;  //Wait for ~88 zeroes
				else NextStateMon = QM0;
			 end
		QM1: begin
				if (SerialData == 1'b1) begin          //Look for a start bit to turn on RAM
					NextStateMon = QM2;
				end else begin
					NextStateMon = QM1;
				end
			 end
		QM2: begin
				if (DMPStart || (SerialDataDly == 0)) NextStateMon = QM3;   //Keep RAM on until ~88 zeroes go by
				else NextStateMon = QM2;
			 end
		QM3: begin
				NextStateMon = QM4;
			 end
		QM4: begin
				if (StateRAM == QR0) NextStateMon = QM1;
				else NextStateMon = QM4;
			 end
		default: NextStateMon = QM0;
	endcase
end

always @ (posedge Clock)
begin
	if (Reset) begin
		StateMon <= QM0;
		RAMStart <= 1'b0;
		RAMStop <= 1'b0;
	end else begin
		StateMon <= NextStateMon;
		case (StateMon)
			QM1: begin
					if (SerialData == 1'b1) RAMStart <= 1; //This signal is one clock cycles late, hence the Delay shift register below.
					RAMStop <= 1'b0;
				 end
			QM2: begin
					RAMStart <= 1'b0;
				 end
			QM3: begin
					RAMStop <= 1'b1;
				 end
		endcase
	end
end


//State machine to write the chip's output into block RAM.  
//Definition of states:
parameter [1:0] QR0= 2'b00;  //Wait for a start signal
parameter [1:0] QR1= 2'b01;  //Shift in 8 bits
parameter [1:0] QR2= 2'b10;  //Write a byte to RAM

reg [1:0] StateRAM, NextStateRAM;
reg RAMStart, RAMStop;  //Control signals from outside this process
reg [15:0] WriteAdr;
reg [7:0] RAMbyte;  //One byte to write to memory
wire [7:0] doa;      //RAM data out from port a
reg [2:0] CntByte;  //Counter for filling one byte
reg ena;            //Read enable memory port a
reg wea;            //Memory write enable
reg [2:0] Delay;    //Delay the incoming data by three clock cycles

always @ (StateRAM or RAMStart or RAMStop or CntByte or WriteAdr)
begin
  case (StateRAM)
	QR0: begin
			if (RAMStart && WriteAdr<16'hf000) NextStateRAM = QR1;  //Don't start if packet might overflow RAM
			else NextStateRAM = QR0;
			ena = 0;
			wea = 0;
	     end
	QR1: begin
			if (WriteAdr == 16'hFFFF) NextStateRAM = QR0;  //Avoid overflow and overwriting of the RAM
	      else if (CntByte == 7) NextStateRAM = QR2;
			else NextStateRAM = QR1;
			ena = 0;
			wea = 0;
		 end
	QR2: begin
			if (RAMStop) NextStateRAM = QR0;
			else NextStateRAM = QR1;
			ena = 1;
			wea = 1;      //Write the byte into memory on the next clock edge
		 end
	default: begin NextStateRAM = QR0; ena = 0; wea = 0; end
  endcase
end

always @ (posedge Clock)
begin
  if (Reset) begin
    StateRAM <= QR0;
	WriteAdr <= 0;
  end else if (DMPDone) begin
	WriteAdr <= 0;               //Clear the buffer after it has been written out
  end else begin
	Delay <= {Delay[1:0], SerialData};
	StateRAM <= NextStateRAM;
	case (StateRAM)
		QR0: begin
				CntByte <= 0;
			 end
		QR1: begin
				RAMbyte <= {RAMbyte[6:0],Delay[2]};
				CntByte <= CntByte + 1;
			 end
		QR2: begin
				RAMbyte <= {RAMbyte[6:0],Delay[2]};
				CntByte <= CntByte + 1;
				WriteAdr <= WriteAdr + 1;
			 end
	endcase
  end
end


//State machine to dump the RAM to the UART
//Definition of states:
parameter [8:0] QD0= 9'b000000001;  //Wait for a start signal
parameter [8:0] QD1= 9'b000000010;  //Wait for the Tx to be ready
parameter [8:0] QD2= 9'b000000100;  //Write the first (most significant) byte of the header
parameter [8:0] QD3= 9'b000001000;  //Wait for the Tx to be ready
parameter [8:0] QD4= 9'b000010000;  //Write the second byte of the header
parameter [8:0] QD5= 9'b000100000;  //Skip one clock cycle to give time for the UART to catch the header
parameter [8:0] QD6= 9'b001000000;  //Wait for the Tx to be ready
parameter [8:0] QD7= 9'b010000000;  //Write a byte from RAM to Tx
parameter [8:0] QD8= 9'b100000000;  //Signal done

reg enb;
reg [15:0] ReadAdr;
wire [7:0] dob;
reg [8:0] StateDMP, NextStateDMP;
reg [2:0] CntWt;
parameter [2:0] Dly= 3'h2; //Set length of time to assert TxD_start

always @ (StateDMP or DMPStart or TxD_busy or ReadAdr or WriteAdr or dob or CntWt)
begin
	case (StateDMP)
		QD0: begin
				if (DMPStart) NextStateDMP = QD1;
				else NextStateDMP = QD0;
				enb = 0;
				TxD_data = dob;
			 end
		QD1: begin
				enb = 0;
				TxD_data = WriteAdr[15:8];
				if (TxD_busy) NextStateDMP = QD1;
				else NextStateDMP = QD2;
			 end
		QD2: begin
				enb = 0;
				TxD_data = WriteAdr[15:8];
				if (CntWt==Dly) NextStateDMP=QD3;
				else NextStateDMP = QD2;
			 end
		QD3: begin
				enb = 0;
				TxD_data = WriteAdr[15:8];
				if (TxD_busy) NextStateDMP = QD3;
				else NextStateDMP = QD4;
			 end
		QD4: begin
			    enb = 0;
				TxD_data = WriteAdr[7:0];
				if (CntWt==Dly) NextStateDMP = QD5;
				else NextStateDMP = QD4;
			 end
		QD5: begin
				enb = 0;
				TxD_data = WriteAdr[7:0];
				if (WriteAdr!=0) NextStateDMP = QD6;
				else NextStateDMP = QD8;
			 end
		QD6: begin
				if (TxD_busy) NextStateDMP = QD6;
				else NextStateDMP = QD7;
				enb = 1;    //get the next byte from memory
				TxD_data = dob;
			 end
		QD7: begin
				enb = 0;
				if (CntWt!=Dly) NextStateDMP = QD7;
				else begin
					if (ReadAdr == WriteAdr) NextStateDMP = QD8;
					else NextStateDMP = QD6;
				end
				TxD_data = dob;
			 end
		QD8: begin
				enb = 0;
				NextStateDMP = QD0;
				TxD_data = dob;
			 end
		default: begin
					enb = 0;
					NextStateDMP = QD0;
					TxD_data = dob;
				 end
	endcase
end

always @ (posedge Clock)
begin
	if (Reset) begin
		StateDMP <= QD0;
		ReadAdr <= 0;
		TxD_start <= 0;
	end else begin
		StateDMP <= NextStateDMP;
		case (StateDMP)
			QD0: begin
					DMPDone <= 1'b0;
				 end
			QD1: begin
					TxD_start <= 0;
					CntWt <= 0;
				 end
			QD2: begin
					TxD_start <= 1;
					CntWt <= CntWt + 1;
//					$display("%g\t Write address=%h",$time,WriteAdr);
				 end
			QD3: begin
					CntWt <= 0;
					TxD_start <= 0;
				 end
			QD4: begin
					CntWt <= CntWt + 1;
					TxD_start <= 1;
				 end
			QD5: begin
					TxD_start <= 0;
				 end
			QD6: begin
					CntWt <= 0;
					TxD_start <= 0;
				 end
			QD7: begin
					CntWt <= CntWt + 1;
					TxD_start <= 1;
					if (CntWt==0) ReadAdr <= ReadAdr + 1;
				 end
			QD8: begin
					TxD_start <= 0;
					DMPDone <= 1'b1;
					ReadAdr <= 0;
				 end
		endcase
	end
end

DataOutRAM DataOutRAM_U(Clock, Clock, ena, enb, wea, WriteAdr, ReadAdr, RAMbyte, doa, dob);   //Instantiate the memory here

endmodule
	
//Dual ported RAM with one write port:
module DataOutRAM (clka, clkb, ena, enb, wea, addra, addrb, dia, doa, dob);
input clka, clkb;
input wea;                  //Write enable
input ena, enb;             //Primary and secondary read enables
input [15:0] addra, addrb;  //Addresses for the primary and secondary ports
input [7:0] dia;           	//Input data register to write to memory
output [7:0] doa, dob;     	//Output registers for the two ports
reg [7:0] RAM [65535:0];    //Memory array
reg [7:0] dob, doa;

always @(posedge clka)
begin
	if (ena)
	begin
		if (wea) begin
			RAM[addra]<=dia;  
			doa <= dia;
		end else doa <= RAM[addra];        //Write first, then read
//		$display("%g\t RAM:  write %b to address %h",$time,dia,addra);
	end
end

always @(posedge clkb)
begin
	if (enb)
	begin
		dob <= RAM[addrb];
//		$display("%g\t RAM: read %b from address %h",$time,RAM[addrb],addrb);
	end
end
endmodule
