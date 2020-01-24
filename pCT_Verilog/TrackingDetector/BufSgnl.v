// Generate a signal each time a front-end ASIC																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																							// Monitor the signals from the tracker board readout indicating when ASIC front-end
// buffer has been freed up, and relay the information to the event builder to be
// used in trigger management.  V boards only have to be concerned with a single 
// signal, but on T boards the signals from the two FPGAs must be multiplexed.
// onto a single line.  Only on the even FPGA of the T board do we get dual inputs.
// On the odd FPGA the output of this routine is not even used.
// R. Johnson,  September 8, 2013
// March 18, 2014, rewritten to simplify the scheme considerably and make it more robust.
// April 28, 2014, rewrote the queue state machine, hopefully to avoid timing errors

module BufSgnl(Stream, Signal0, Signal1, Reset, Clock, Address);
input Reset;				// Synchronous reset, one clock in length
input Clock;				// 100 MHz system clock
input [3:0] Address;		// FPGA address
input Signal0;				// Undo buffer signal from the FPGA in which the code is running
input Signal1;				// Undo buffer signal from the other (odd) FPGA when running 
							// in an even FPGA on a T board.  (not used in odd FPGA)
output Stream;				// Serial output stream

// Output stream format: 110 for V board or even FGPA, 101 for T board.
reg Stream;

// The input signals have to be queued, because they may arrive during
// processing of the previous signals, especially for T boards.
reg [3:0] NQueue, WritePtr, ReadPtr;
reg Decrement;
reg [15:0] Queue;
reg Prev0, Prev1;

//Machine to queue the signals
parameter [3:0] WaitQu = 4'b0001;
parameter [3:0] EvnFQu = 4'b0010;
parameter [3:0] OddFQu = 4'b0100;
parameter [3:0] BothQu = 4'b1000;
reg [3:0] StateQu, NextStateQu;

assign UnDoBuf0 = Signal0 & !Prev0;   //Just in case the signal lasts more than one clock cycle
assign UnDoBuf1 = Signal1 & !Prev1;
always @ (StateQu or UnDoBuf0 or UnDoBuf1) begin
	case (StateQu)
		WaitQu:	begin
					if (UnDoBuf0) begin
						if (UnDoBuf1) NextStateQu = BothQu;
						else NextStateQu = EvnFQu;
					end else if (UnDoBuf1) begin
						NextStateQu = OddFQu;
					end else begin
						NextStateQu = WaitQu;
					end
				end
		EvnFQu:	begin
					if (UnDoBuf1) NextStateQu = OddFQu;
					else NextStateQu = WaitQu;
				end
		OddFQu:	begin
					if (UnDoBuf0) NextStateQu = EvnFQu;
					else NextStateQu = WaitQu;
				end
		BothQu:	begin
					NextStateQu = WaitQu;
				end
		default:	begin
						NextStateQu = WaitQu;
					end
	endcase
end

wire [3:0] WritePtrP1;
assign WritePtrP1 = WritePtr + 1;
always @ (posedge Clock) begin
	if (Reset) begin
		NQueue <= 0;
		WritePtr <= 0;
		StateQu <= WaitQu;
		Prev0 <= 1'b0;
		Prev1 <= 1'b0;
	end else begin
		Prev0 <= Signal0;
		Prev1 <= Signal1;
		StateQu <= NextStateQu;
		if (StateQu != WaitQu) begin
			$display("%g\t BufSgnl %d StateQu %b: UnDoBuf=%b %b, Decrement=%b, Queue=%b, NQueue=%d, WritePtr=%d, ReadPtr=%d",$time,Address,StateQu,UnDoBuf0,UnDoBuf1,Decrement,Queue,NQueue,WritePtr,ReadPtr); 
		end
		case (StateQu) 
			WaitQu:	begin
						if (Decrement) NQueue <= NQueue - 1;
					end
			EvnFQu:	begin
						Queue[WritePtr] <= 1'b0;
						if (!Decrement) NQueue <= NQueue + 1;
						WritePtr <= WritePtr + 1;
					end
			OddFQu:	begin
						Queue[WritePtr] <= 1'b1;
						if (!Decrement) NQueue <= NQueue + 1;
						WritePtr <= WritePtr + 1;
					end
			BothQu: begin
						Queue[WritePtr] <= 1'b0;
						Queue[WritePtrP1] <= 1'b1;
						WritePtr <= WritePtr + 2;
						if (!Decrement) NQueue <= NQueue + 2;
						else NQueue <= NQueue + 1;
					end
		endcase
	end
end

// always @ (NQueue) begin
	// $display("%g\t BufSgnl %d, NQueue=%d, Queue=%b, WritePtr=%d, ReadPtr=%d",$time,Address,NQueue,Queue,WritePtr,ReadPtr);
// end

//State machine to send the serial stream

parameter [3:0] Wait = 4'b0001;		//Wait for the queue not to be empty
parameter [3:0] Strt = 4'b0010;		//Send a start bit
parameter [3:0] Scnd = 4'b0100;		//Send a second bit
parameter [3:0] Thrd = 4'b1000;		//Send the last bit

reg [3:0] State, NextState;

always @ (State or NQueue or ReadPtr or Queue) begin
	case (State)
		Wait:	begin
					if (NQueue != 0) NextState = Strt;
					else NextState = Wait;
					Stream = 1'b0;
				end
		Strt:	begin
					NextState = Scnd;
					Stream = 1'b1;
				end
		Scnd:	begin
					NextState = Thrd;
					Stream = !Queue[ReadPtr];
				end
		Thrd:	begin
					NextState = Wait;
					Stream = Queue[ReadPtr];
				end
		default:	begin
						NextState = Wait;
						Stream = 1'b0;
					end
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		State <= Wait;
		ReadPtr <= 0;
		Decrement <= 1'b0;
	end else begin
		State <= NextState;
		case (State) 
			Scnd:	begin
						Decrement <= 1'b1;
					end
			Thrd:	begin
						ReadPtr <= ReadPtr + 1;
						Decrement <= 1'b0;
						if (Address==10) $display("%g\t BufSgnl %d, sending signal 1%b%b.",$time,Address,!Queue[ReadPtr],Queue[ReadPtr]);
					end		
		endcase
	end
end

endmodule
