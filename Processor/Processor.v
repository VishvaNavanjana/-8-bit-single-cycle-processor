//timescale
`timescale  1ns/100ps

//testbench
module cpu_tb;

    reg CLK, RESET;
    wire [31:0] INSTRUCTION;
	  wire read,write,busywait;
	  wire [7:0] address,readdata,writedata;
    wire [31:0] PC;

    wire memBusywait,memRead,memWrite;
    wire [31:0] memWritedata,memReaddata;
    wire [5:0] memAddress;
    
    /* 
    ------------------------
     SIMPLE INSTRUCTION MEM
    ------------------------
    */
    
    // TODO: Initialize an array of registers (8x1024) named 'instr_mem' to be used as instruction memory
    //reg [7:0] instr_mem [1023:0];
    // TODO: Create combinational logic to support CPU instruction fetching, given the Program Counter(PC) value 
    //       (make sure you include the delay for instruction fetching here)
    //assign #2 INSTRUCTION={instr_mem[PC+3], instr_mem[PC+2], instr_mem[PC+1], instr_mem[PC]};
    
    // Read the instruction memory . Memory Read has 2 time unit delay
    //assign #2 INSTRUCTION[7:0]=instr_mem[PC];
    //assign #2 INSTRUCTION[15:8]=instr_mem[PC+1]; 
    //assign #2 INSTRUCTION[23:16]=instr_mem[PC+2];
    //assign #2 INSTRUCTION[31:24]=instr_mem[PC+3];
    //initial
    //begin
        // Initialize instruction memory with the set of instructions you need execute on CPU
        
        // METHOD 1: manually loading instructions to instr_mem
        //{instr_mem[10'd3], instr_mem[10'd2], instr_mem[10'd1], instr_mem[10'd0]} = 32'b00000000000001000000000000000101;
        //{instr_mem[10'd7], instr_mem[10'd6], instr_mem[10'd5], instr_mem[10'd4]} = 32'b00000000000000100000000000001001;
        //{instr_mem[10'd11], instr_mem[10'd10], instr_mem[10'd9], instr_mem[10'd8]} = 32'b00000010000001100000010000000010;
        
        // METHOD 2: loading instr_mem content from instr_mem.mem file
        //$readmemb("programmer/instr_mem.mem", instr_mem);
    //end
    
    /* 
    -----
     CPU
    -----
    */
    wire imemBusywait,imemRead;
    wire [127:0] readBlock;
    wire [5:0] imemAddress;

    cpu cu(INSTRUCTION,CLK,RESET,readdata,busywait,PC,writedata,address,write,read);
    instruction_cache ic(PC,CLK,INSTRUCTION,RESET,imemBusywait,busywait,imemRead,readBlock,imemAddress);
    instruction_memory im(CLK,imemRead,imemAddress,readBlock,imemBusywait);
    dcache dc(CLK,RESET,read,write,address,writedata,readdata,busywait,memBusywait,memRead,memWrite,memWritedata,memReaddata,memAddress);
    data_memory MY(CLK,RESET,memRead,memWrite,memAddress,memWritedata,memReaddata,memBusywait);

    initial
    begin
    
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("Group_28.vcd");
		    $dumpvars(0, cpu_tb);
        
        CLK = 1'b0;
        RESET = 1'b0;
        
        // TODO: Reset the CPU (by giving a pulse to RESET signal) to start the program execution
        // initilaize the PC = 0
        #3 RESET = 1'b1;
		
				#5 RESET = 1'b0;
        // finish simulation after some time
        #2600
        $finish;
        
    end 
    
    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule





/*
Module	: 256x8-bit instruction memory (16-Byte blocks)
Author	: Isuru Nawinne
Date		: 10/06/2020

Description	:

This file presents a primitive instruction memory module for CO224 Lab 6 - Part 3
This memory allows instructions to be read as 16-Byte blocks
*/

module instruction_memory(
	clock,
	read,
    address,
    readinst,
	busywait
);
input				clock;
input				read;
input[5:0]			address;
output reg [127:0]	readinst;
output	reg			busywait;

reg readaccess;

//Declare memory array 1024x8-bits 
reg [7:0] memory_array [1023:0];

//Initialize instruction memory
initial
begin
	busywait = 0;
	readaccess = 0;
	
    // Sample program given below. You may hardcode your software program here, or load it from a file:
    //{memory_array[10'd3],  memory_array[10'd2],  memory_array[10'd1],  memory_array[10'd0]}  = 32'b00000000000001000000000000011001; // loadi 4 #25
    //{memory_array[10'd7],  memory_array[10'd6],  memory_array[10'd5],  memory_array[10'd4]}  = 32'b00000000000001010000000000100011; // loadi 5 #35
    //{memory_array[10'd11], memory_array[10'd10], memory_array[10'd9],  memory_array[10'd8]}  = 32'b00000010000001100000010000000101; // add 6 4 5
    //{memory_array[10'd15], memory_array[10'd14], memory_array[10'd13], memory_array[10'd12]} = 32'b00000000000000010000000001011010; // loadi 1 90
    //{memory_array[10'd19], memory_array[10'd18], memory_array[10'd17], memory_array[10'd16]} = 32'b00000011000000010000000100000100; // sub 1 1 4

    $readmemb("programmer/instr_mem.mem", memory_array);
end

//Detecting an incoming memory access
always @(read)
begin
    busywait = (read)? 1 : 0;
    readaccess = (read)? 1 : 0;
end

//Reading
always @(posedge clock)
begin
	if(readaccess)
	begin
		readinst[7:0]     = #40 memory_array[{address,4'b0000}];
		readinst[15:8]    = #40 memory_array[{address,4'b0001}];
		readinst[23:16]   = #40 memory_array[{address,4'b0010}];
		readinst[31:24]   = #40 memory_array[{address,4'b0011}];
		readinst[39:32]   = #40 memory_array[{address,4'b0100}];
		readinst[47:40]   = #40 memory_array[{address,4'b0101}];
		readinst[55:48]   = #40 memory_array[{address,4'b0110}];
		readinst[63:56]   = #40 memory_array[{address,4'b0111}];
		readinst[71:64]   = #40 memory_array[{address,4'b1000}];
		readinst[79:72]   = #40 memory_array[{address,4'b1001}];
		readinst[87:80]   = #40 memory_array[{address,4'b1010}];
		readinst[95:88]   = #40 memory_array[{address,4'b1011}];
		readinst[103:96]  = #40 memory_array[{address,4'b1100}];
		readinst[111:104] = #40 memory_array[{address,4'b1101}];
		readinst[119:112] = #40 memory_array[{address,4'b1110}];
		readinst[127:120] = #40 memory_array[{address,4'b1111}];
		busywait = 0;
		readaccess = 0;
	end
end
 
endmodule


module instruction_cache(pc,clk,instruction,reset,memBusywait,cpuBusywait,read,readBlock,address);
    input memBusywait;
    input reset,clk;
    input [31:0] pc;
    input [127:0] readBlock;

    output reg [5:0] address;
    output reg read;
    output reg cpuBusywait;
    output reg [31:0] instruction;

    reg [1:0] offset;
    reg [2:0] index;
    reg [2:0] tag;
    reg hit,tagMatched,validBit;
    reg [131:0] icache_storage [7:0];

    //resetting cache
    integer i;
    always @ (posedge clk)
    begin
      if(reset==1)begin
        for(i=0;i<8;i=i+1)begin
          icache_storage[i] = 132'd0;
        end
      end  
    end


    //decoding the address
    always @ (pc,currentBlock)
    begin
      cpuBusywait = 1;
      offset = pc[3:2];
      index = pc[6:4];
      tag = pc[9:7];
    end


    reg [131:0] currentBlock;

    always @ (index)
    #1 begin
      currentBlock = icache_storage[index];
    end

    always @ (currentBlock)
    begin
      validBit = currentBlock[131];
    end

    //checking whther its a hit or a miss
    always @ (currentBlock,pc)
    #0.9 begin
      if(tag == currentBlock[130:128])begin
        tagMatched = 1;
      end else begin
        tagMatched = 0;
      end

      if(validBit && tagMatched)begin
        hit = 1;
      end else begin
        hit = 0;
      end
    end

    //selecting instruction
    reg [31:0] selectedInstruction;
    always @ (offset,currentBlock)
     #1 begin
      case (offset)
			2'd0 : selectedInstruction = currentBlock[31:0];
			2'd1 : selectedInstruction = currentBlock[63:32];
			2'd2 : selectedInstruction = currentBlock[95:64];
			2'd3 : selectedInstruction = currentBlock[127:96];
		endcase
    end


    //operation if its a hit
    always @ (*)
    begin
      if(hit)begin
        //give the selected instruction to the cpu
        instruction = selectedInstruction;
        //set the busywait signal to low
        cpuBusywait = 0;
      end
    end

    //if its a miss
    always @ (hit)
    begin
      if(!hit)begin
        read = 1;
        //memAddress = {tag, index};
        address = {tag,index};
      end else begin
        read = 0;
      end
    end

    //wtiting to the cache
    reg [131:0] tempBlock;

    always @ (posedge clk)
    #1 begin
      if(!memBusywait && !hit)begin
        tempBlock[127:0] = readBlock;
        tempBlock[130:128] = tag;
        tempBlock[131] = 1;
        currentBlock = tempBlock;
      end
    end

endmodule




/*
Module	: 256x8-bit data memory (4-Byte blocks)
Author	: Isuru Nawinne
Date    	: 30/05/2020

Description	:

This file presents a primitive data memory module for CO224 Lab 6 - Part 2
This memory allows data to be read and written as 4-Byte blocks
*/

module data_memory(
	clock,
    reset,
    read,
    write,
    address,
    writedata,
    readdata,
	busywait
);
input				clock;
input           	reset;
input           	read;
input           	write;
input[5:0]      	address;
input[31:0]     	writedata;
output reg [31:0]	readdata;
output reg      	busywait;

//Declare memory array 256x8-bits 
reg [7:0] memory_array [255:0];

//Detecting an incoming memory access
reg readaccess, writeaccess;
always @(read, write)
begin
	busywait = (read || write)? 1 : 0;
	readaccess = (read && !write)? 1 : 0;
	writeaccess = (!read && write)? 1 : 0;
end

//Reading & writing
always @(posedge clock)
begin
	if(readaccess)
	begin
		readdata[7:0]   = #40 memory_array[{address,2'b00}];
		readdata[15:8]  = #40 memory_array[{address,2'b01}];
		readdata[23:16] = #40 memory_array[{address,2'b10}];
		readdata[31:24] = #40 memory_array[{address,2'b11}];
		busywait = 0;
		readaccess = 0;
	end
	if(writeaccess)
	begin
		memory_array[{address,2'b00}] = #40 writedata[7:0];
		memory_array[{address,2'b01}] = #40 writedata[15:8];
		memory_array[{address,2'b10}] = #40 writedata[23:16];
		memory_array[{address,2'b11}] = #40 writedata[31:24];
		busywait = 0;
		writeaccess = 0;
	end
end


//Reset memory
integer i;
always @(posedge reset)
begin
    if (reset)
    begin
        for (i=0;i<256; i=i+1)
            memory_array[i] = 0;
        
        busywait = 0;
		readaccess = 0;
		writeaccess = 0;
    end
end

endmodule

//data cache module
module dcache (
    clock,
    reset,
    read,
    write,
    address,
    writedata,
    readdata,
	busywait,

    memBusywait,
    memRead,
    memWrite,
    memWritedata,
    memReaddata,
    memAddress
);
    //defininig ports to the module
    input memBusywait,clock,write,read,reset;
    input [31:0] memReaddata;
    input [7:0] address,writedata;

    output reg memRead,memWrite,busywait;
    output reg [31:0] memWritedata;
    output reg [7:0] readdata;
    output reg [5:0] memAddress;

    //cache memory
    reg [36:0] cache_storage [7:0];
    reg valid,dirty;
    reg [2:0] index;
    reg [1:0] offset;
    reg [2:0] tag;
    reg hit;
    reg [7:0] selectedWord;
    reg tagMatched;

    //resetting cache
    integer i;
    always @ (posedge clock)
    begin
      if(reset == 1)begin
        for(i=0;i<8;i=i+1)begin
          cache_storage[i] = 37'd0;
        end
      end  
    end

  //to dump cache memory
  integer count;
	initial
    begin
        $dumpfile("Group_28.vcd");
				$dumpvars(0, dcache);
        for(count = 0; count < 8; count++)begin
            $dumpvars(1, cache_storage[count]);
		end
    end

    //generating the busywait signal for cpu
    always @ (read,write,address)
    begin
      busywait = (read || write) ? 1'b1 : 1'b0;
    end

    //decoding the address
    always @ (address)
    begin
      index = address[4:2];
      offset = address[1:0];
      tag = address[7:5];
    end

    //to hold a block of cache
    reg [36:0] currentBlock;

    //extracting the block
    always @ (readdata,writedata,cache_storage[index])
    #1 begin
      currentBlock = cache_storage[index];
    end

    //extracting valid bit and dirty bit
    always @ (currentBlock)begin
      valid = currentBlock[36];
      dirty = currentBlock[35];
    end

    //selecting data word
    always @ (offset,currentBlock)
     #1 begin
      case (offset)
			2'd0 : selectedWord = currentBlock[7:0];
			2'd1 : selectedWord = currentBlock[15:8];
			2'd2 : selectedWord = currentBlock[23:16];
			2'd3 : selectedWord = currentBlock[31:24];
		endcase
    end

    //checking whether its a hit or not
    always @ (address,currentBlock)
    #0.9 begin
      if(tag == currentBlock[34:32])begin
        tagMatched = 1;
      end else begin
        tagMatched = 0;
      end
      //creating the hit signal
      hit = tagMatched && valid;
    end

    //operation if its a hit
    always @ (*)
    begin
      if(hit && read)begin
        //give the selected word to the cpu
        readdata = selectedWord;
        //set the busywait signal to low
        busywait = 0;
      end
      if(hit && write)begin
        //set the busywait signal to low
        busywait = 0;
      end
    end

    //writing when there is a hit
    always @ (posedge clock)
    #1 begin
       if(hit && write)begin
        //getting the word to the right position of the cache block
        case (offset)
					2'd0 : currentBlock[7:0] 	= writedata;
					2'd1 : currentBlock[15:8]	= writedata;
					2'd2 : currentBlock[23:16]	= writedata;
					2'd3 : currentBlock[31:24]	= writedata;
		endcase
        //setting validbit and dirtybit as high
        currentBlock[35] = 1;
        currentBlock[36] = 1;
        //store the current block to the cache memory
        cache_storage[index] = currentBlock;
      end
    end


    /* Cache Controller FSM Start */

    parameter IDLE = 3'b000, MEM_READ = 3'b001, MEM_WRITE = 3'b010, CACHE_UPDATE = 3'b011;
    reg [2:0] state, next_state;
    reg readDone; //this sets to 1 if we finished reading from memory

    // combinational next state logic
    always @(*)
    begin
        case (state)
            IDLE:
                if ((read || write) && !dirty && !hit)  
                    next_state = MEM_READ;
                else if ((read || write) && dirty && !hit)
                    next_state = MEM_WRITE;
                else
                    next_state = IDLE;
            
            MEM_READ:
                if (!memBusywait)
                    next_state = CACHE_UPDATE;
                else    
                    next_state = MEM_READ;

            MEM_WRITE:
                if (!memBusywait)
                    next_state = MEM_READ;
                else    
                    next_state = MEM_WRITE;   

            CACHE_UPDATE:
                next_state = IDLE;        
        endcase
    end

    

    // combinational output logic
    always @(state)
    begin
        case(state)
            IDLE:
            begin
                readDone = 0;
                memRead = 0;
                memWrite = 0;
                memAddress = 6'dx;
                memWritedata = 32'dx;
                busywait = 0;
            end
         
            MEM_READ: 
            begin
                memRead = 1;
                memWrite = 0;
                //tag is got grom the address which comes from the cpu
                memAddress = {tag, index};
                memWritedata = 32'dx;
                busywait = 1;
            end

            MEM_WRITE:
            begin
                memRead = 0;
                memWrite = 1;
                //tag is got from the currentBlock
                memAddress = {currentBlock[34:32], index};
                memWritedata = currentBlock[31:0];
                busywait = 1;
            end

            CACHE_UPDATE:
            begin
              readDone = 1;
              memRead = 0;
              memWrite = 0;
              memAddress = 6'dx;
              memWritedata = 32'dx;
              busywait = 1;
            end  

        endcase
    end

    // sequential logic for state transitioning 
    always @(posedge clock, reset)
    begin
        if(reset)
            state = IDLE;
        else
            state = next_state;
    end

    /* Cache Controller FSM End */
    
    reg [36:0] tempory;
    //writing to the cache
    always @ (posedge clock,memBusywait)
    #1 begin
      if(readDone && !memBusywait && (write || read))begin
        tempory [31:0] = memReaddata;
        tempory [34:32] = tag;
        tempory [35] = 1'b0;  //dirty bit is @35
        tempory [36] = 1'b1;  //valid bit is @36
        //storing to the cache memory
        cache_storage[index] = tempory;

      end
    end



endmodule



//cpu module
module cpu(INSTRUCTION,CLK,RESET,READDATA,BUSYWAIT,PC,WRITEDATA,ADDRESS,WRITE,READ);
    //ports of the module
    input [31:0] INSTRUCTION;
    input CLK,RESET;
    input [7:0] READDATA;
    input BUSYWAIT;

    output reg [31:0] PC;
    output [7:0] WRITEDATA,ADDRESS;
    output WRITE,READ;

    //internal wires
    wire isBeqOperation; 
    wire isJumpOperation;
    wire ZERO;
    wire selectBeq;

    wire [7:0] jumpAddress;
    //getting the address where to jump
    assign jumpAddress = INSTRUCTION[23:16]; 

    //this is for the next pc v
    reg [31:0] adjacent_PC;

    wire [31:0] next_PC;

    //instantiate controUnit module
    controlUnit CU(INSTRUCTION,CLK,RESET,isBeqOperation,isJumpOperation,ZERO,READDATA,WRITEDATA,READ,WRITE,ADDRESS,BUSYWAIT);

    //check whether the BEQ operation is satisfied
    and(selectBeq,isBeqOperation,ZERO);

    //get the next PC
    getNextPC nxtPc(adjacent_PC,jumpAddress,selectBeq,isJumpOperation,next_PC);

    always @ (posedge CLK)
    begin
        //if RESET is high at the posedge of the clk
        if(RESET == 1'b1)begin
          PC = #1 32'h00000000;
        end
        else if(BUSYWAIT==1'b0) begin
            //taking the next pc  
          PC = #1 next_PC;
        end
    end

        //this will trigger whenever PC changes
        always @ (PC)
        begin
          adjacent_PC = #1 PC+32'h00000004;
        end

endmodule


module controlUnit (INSTRUCTION,CLK,RESET,isBeqOperation,isJumpOperation,ZERO,READDATA,WRITEDATA,READ,WRITE,ADDRESS,BUSYWAIT);

    //ports to the module
    input [31:0] INSTRUCTION;
    input RESET,CLK,BUSYWAIT;
    input [7:0] READDATA;

    output reg [7:0] WRITEDATA,ADDRESS;
    output reg READ,WRITE;

    //to check whether the operation is beq or jump
    output reg isBeqOperation,isJumpOperation;

    //ZERO signal
    output ZERO;

    //for the alu
    wire [7:0] ALURESULT;
    reg [2:0] ALUOP;
    wire aluZERO;

    //for the reg_file
    wire [7:0] REGOUT1,REGOUT2;
    reg [2:0] READREG1,READREG2,WRITEREG;

    //to separate the operation
    reg [7:0] OPCODE;

    //get the immediate value
    reg [7:0] IMMEDIATE;

    //set WRITEENABLE as 1'b1
    reg WRITEENABLE = 1'b1;

    //setting write signal to regfile
    reg writeEN;
    always @ (WRITEENABLE,BUSYWAIT)
    begin
      if(WRITEENABLE==1'b1 & BUSYWAIT==1'b0)begin
      writeEN = 1;
      end else begin
      writeEN = 0;
      end
    end

    //these are the selection in 2 muxes
    //s1 is the first mux (Left)
    //s2 is the second mux (Right)
    reg s1,s2;

    //writeDataSelection
    //when is 1 write the data which is coming from the data memory
    //when is 0 write the data which is coming from the alu
    reg writeDataSelection;

    reg [7:0] WRITE2REGISTER;



    //output data from 2 muxes
    //m1OUT is the output from the First mux(Left) 
    //m2OUT is the output from the Second mux(Right)
    reg [7:0] m1OUT,m2OUT; 

    //to get the 2s Complemented value
    reg [7:0] TwosComplent;

    always @ (INSTRUCTION)
    begin
       //getting the opcode from the instruction
        OPCODE = INSTRUCTION[31:24];
        READREG1 = INSTRUCTION[10:8];   //getting the address 1 to read from the reg_file
        READREG2 = INSTRUCTION[2:0];   //getting the address 1 to read from the reg_file
        WRITEREG = INSTRUCTION[18:16]; //getting the write address to write in reg_file
        IMMEDIATE = INSTRUCTION[7:0];   //getting the immediate value
    end
       
  

    //instantiate reg_file module
    reg_file rf(WRITE2REGISTER,REGOUT1,REGOUT2,WRITEREG,READREG1,READREG2,writeEN,CLK,RESET);

    //instantiate alu module
    alu A(REGOUT1,m2OUT,ALURESULT,ALUOP,aluZERO);

    //to get the address
    always @ (ALURESULT)
    begin
      ADDRESS = ALURESULT;
    end

    //getting aluZERO value to ZERO signal
    assign ZERO = aluZERO;
    
    //setting zero at the begining
    always @ (INSTRUCTION)
    begin
      READ = 1'b0;
      WRITE = 1'b0;
    end
    



    //selecting selections on mux
    //ALOUP is found 
    always @ (INSTRUCTION)
     #1 begin //latency for decoding
            case (OPCODE)
               //for loadi
               8'd0 :begin
                 READ = 1'b0;
                 WRITE = 1'b0;
                 WRITEENABLE = 1'b1;
                 writeDataSelection = 1'b0;
                 //selections for muxes
                 s1=1'b0;
                 s2=1'b1;
                 isBeqOperation = 1'b0;
                 isJumpOperation = 1'b0;
                 ALUOP = 3'b000; //setting Alu operation
               end
               
               //for mov 
               8'd1 :begin
                 READ = 1'b0;
                 WRITE = 1'b0;
                 WRITEENABLE = 1'b1;
                 writeDataSelection = 1'b0;
                 //selections for muxes
                 s1=1'b1;
                 s2=1'b0;
                 isBeqOperation = 1'b0;
                 isJumpOperation = 1'b0;
                 ALUOP = 3'b000; //setting Alu operation
               end

               //for add
               8'd2 :begin
                 READ = 1'b0;
                 WRITE = 1'b0;
                 WRITEENABLE = 1'b1;
                 writeDataSelection = 1'b0;
                 //selections for muxes
                 s1=1'b1;
                 s2=1'b0;
                 isBeqOperation = 1'b0;
                 isJumpOperation = 1'b0;
                 ALUOP = 3'b001; //setting Alu operation
               end

               //for sub
               8'd3 :begin
                 READ = 1'b0;
                 WRITE = 1'b0;
                 WRITEENABLE = 1'b1;
                 writeDataSelection = 1'b0;
                 //selections for muxes 
                 s1=1'b0;
                 s2=1'b0;
                 isBeqOperation = 1'b0;
                 isJumpOperation = 1'b0;
                 ALUOP = 3'b001; //setting Alu operation
               end

               //for and
               8'd4 :begin
                 READ = 1'b0;
                 WRITE = 1'b0;
                 WRITEENABLE = 1'b1;
                 writeDataSelection = 1'b0;
                 //selections for muxes
                 s1=1'b1;
                 s2=1'b0;
                 isBeqOperation = 1'b0;
                 isJumpOperation = 1'b0;
                 ALUOP = 3'b010; //setting Alu operation
               end

               //for or
               8'd5 :begin
                 READ = 1'b0;
                 WRITE = 1'b0;
                 WRITEENABLE = 1'b1;
                 writeDataSelection = 1'b0;
                 //selections for muxes
                 s1=1'b1;
                 s2=1'b0;
                 isBeqOperation = 1'b0;
                 isJumpOperation = 1'b0;
                 ALUOP = 3'b011; //setting Alu operation
               end

               //for j
               8'd6 :begin
                 READ = 1'b0;
                 WRITE = 1'b0;
                 WRITEENABLE = 1'b0;
                 writeDataSelection = 1'b0;
                 //selections for muxes
                 isJumpOperation = 1'b1;
                 isBeqOperation = 1'b0;
                 s1=1'b0;
                 s2=1'b1;
                 ALUOP = 3'b000; //setting Alu operation
               end

               //for beq
               8'd7 :begin
                 READ = 1'b0;
                 WRITE = 1'b0;
                 WRITEENABLE = 1'b0;
                 writeDataSelection = 1'b0;
                 //selections for muxes
                 isBeqOperation = 1'b1;
                 isJumpOperation = 1'b0;
                 s1=1'b0;
                 s2=1'b0;
                 ALUOP = 3'b001; //setting Alu operation
               end

                //for lwd
               8'd8 : begin
                 READ = 1'b1;
                 WRITE = 1'b0;
                 writeDataSelection = 1'b1;
                 WRITEENABLE = 1'b1;
                 isBeqOperation = 1'b0;
                 isJumpOperation = 1'b0;
                 s1=1'b1;
                 s2=1'b0;
                 ALUOP = 3'b000; //setting Alu operation
               end

               //for lwi
               8'd9 : begin
                 READ = 1'b1;
                 WRITE = 1'b0;
                 writeDataSelection = 1'b1;
                 WRITEENABLE = 1'b1;
                 isBeqOperation = 1'b0;
                 isJumpOperation = 1'b0;
                 s1=1'b0;
                 s2=1'b1;
                 ALUOP = 3'b000; //setting Alu operation
               end

               //for swd
               8'd10 : begin
                 READ = 1'b0;
                 WRITE = 1'b1;
                 writeDataSelection = 1'b1;
                 WRITEENABLE = 1'b0;
                 isBeqOperation = 1'b0;
                 isJumpOperation = 1'b0;
                 s1=1'b1;
                 s2=1'b0;
                 ALUOP = 3'b000; //setting Alu operation
               end

               //for swi
               8'd11 : begin
                 READ = 1'b0;
                 WRITE = 1'b1;
                 writeDataSelection = 1'b0;
                 WRITEENABLE = 1'b0;
                 isBeqOperation = 1'b0;
                 isJumpOperation = 1'b0;
                 s1=1'b0;
                 s2=1'b1;
                 ALUOP = 3'b000; //setting Alu operation
               end
             
            endcase
        
    end




    //getting data to write to the register file
    always @ (ALURESULT,writeDataSelection,READDATA)
        begin
          case (writeDataSelection)
            1'b0 : WRITE2REGISTER = ALURESULT;  //write the alu result
            1'b1 : WRITE2REGISTER = READDATA; //get the data from data_memory
          endcase
        end

    always @ (ALURESULT,WRITE,REGOUT1) 
        begin
             WRITEDATA = REGOUT1;   //getiing the data from regout1 to wite to the data memory
        end   

    //getting data from First mux
    always @ (s1,REGOUT2,TwosComplent)
    begin
        case (s1)
            1'b1 : m1OUT = REGOUT2;  //getting the normal output value from the reg_file
            1'b0 : m1OUT = TwosComplent;  //getting the complemented value 
            
        endcase
    end

    //to get the 2s complemented value
    always @ (REGOUT2) begin
      TwosComplent = #1 ~REGOUT2+8'd1;
    end

    //getting data from Second mux
    always @ (s2,IMMEDIATE,m1OUT)
    begin
        case (s2)
            1'b0 : m2OUT = m1OUT;   //getting the output of the 1st mux
            1'b1 : m2OUT = IMMEDIATE;  //getting the immediate value 
        endcase
    end

endmodule



//module to get the next pc
module getNextPC(adjacent_PC,jumpAddress,selectBeq,isJumpOperation,next_PC);

  //ports
  input [31:0] adjacent_PC;
  input [7:0] jumpAddress;
  input selectBeq,isJumpOperation;
  output reg [31:0] next_PC;

  //to hold the extended address
  reg [31:0] extended_address;

  //to hold the value of PC temporily
  reg [31:0] temporyPC;

  always @ (jumpAddress)
  begin
    extended_address[1:0] = 2'b00;  //to shift 2 bits left
    extended_address[9:2] = jumpAddress;    //getting the address

 
    //filling the other most significant bits
	extended_address[31:10] = {22{jumpAddress[7]}};
	
  end 


  always @ (jumpAddress,adjacent_PC)
  #2
  begin
    //get the extended address by adding the adjacent address to it
    extended_address = extended_address + adjacent_PC; 
  end


  always @ (selectBeq,adjacent_PC,extended_address)
  begin
    case (selectBeq)
      1'b1 : temporyPC = extended_address;  //assign the extended 32 bit address
      1'b0 : temporyPC = adjacent_PC; //assign the adjacent_PC
    endcase
  end

  always @ (isJumpOperation,temporyPC,extended_address)
  begin
    case (isJumpOperation)
      1'b1 : next_PC = extended_address;  //assign the extended 32 bit address
      1'b0 : next_PC = temporyPC;   //get the temporyPC as the next_PC value
    endcase
  end



endmodule





module reg_file(IN,OUT1,OUT2,INADDRESS,OUT1ADDRESS,OUT2ADDRESS, WRITE, CLK, RESET);

  initial
    begin
			#5;
			$display("\n\t\t\t==================================================");
			$display("\t\t\t Change of Register Content Starting from Time #5");
			$display("\t\t\t==================================================");
			$display("\t\ttime\treg0\treg1\treg2\treg3\treg4\treg5\treg6\treg7");
			$display("\t\t-------------------------------------------------------");
			$monitor($time,"\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d",regs[0],regs[1],regs[2],regs[3],regs[4],regs[5],regs[6],regs[7]);
			
    end



	//port declaration
  input WRITE,RESET,BUSYWAIT, CLK;	
	input [7:0] IN;
  input [2:0] OUT1ADDRESS,OUT2ADDRESS,INADDRESS;				
	output [7:0] OUT1,OUT2;		
	
		
	
  //8x8 register
	reg [7:0] regs [7:0];	

	integer count;
	
	initial
    begin
        $dumpfile("Group_28.vcd");
				$dumpvars(0, reg_file);
        for(count = 0; count < 8; count++)begin
            $dumpvars(1, regs[count]);
		end
    end
		
	  assign #2 OUT1 = regs[OUT1ADDRESS];
    assign #2 OUT2 = regs[OUT2ADDRESS];
	
		always @(posedge CLK)
		begin

			if(RESET==1'b1)	//Reseting the 8x8 register array
			begin
				#1 for(count=0; count<8 ;count=count+1)
				begin
						regs[count] = 8'd0;
				end
			end

			else if(WRITE==1'b1  & RESET==1'b0)
			#1 begin
				regs[INADDRESS] = IN;	 //Register writing
			end
			 
		end

endmodule

module alu(DATA1, DATA2, RESULT, SELECT,ZERO);

								//port declaration
	input [7:0] DATA1, DATA2;	//7-bit inputs	
	input [2:0] SELECT;			//3-bit input
	output reg [7:0] RESULT;	//7-bit reg output 
  output ZERO;  //zero signal
	
	wire [7:0] ResultFoward, ResultAdd, ResultAnd, ResultOr;	//initializing variables that are used to store results
	
											
	FORWARD  my_forward(DATA2, ResultFoward);	//instance for FORWARD module			
	ADD  my_add(DATA1, DATA2, ResultAdd);		//instance for ADD module	
	AND  my_and(DATA1, DATA2, ResultAnd);		//instance for AND module	
	OR  my_or(DATA1, DATA2, ResultOr);			//instance for OR module	
	
	//this always block runs when DATA1/DATA2/SELECT are changing
	always @(ResultFoward, ResultAdd, ResultAnd, ResultOr, SELECT)	
	begin
		case (SELECT)						//blocking assignment
			3'b000 :	RESULT = ResultFoward;	//if SELECT=0 , RESULT = ResultFoward
			3'b001 :	RESULT = ResultAdd;		//if SELECT=1 , RESULT = ResultAdd
			3'b010 :	RESULT = ResultAnd;		//if SELECT=2 , RESULT = ResultAnd
			3'b011 :	RESULT = ResultOr;		//if SELECT=3 , RESULT = ResultOr
			default :	RESULT = 8'b0000_0000;  //if SELECT=1xxx , RESULT gets zero
		endcase
	end
  
  //getting the zero signal
  nor(ZERO,RESULT[0],RESULT[1],RESULT[2],RESULT[3],RESULT[4],RESULT[5],RESULT[6],RESULT[7]);
	
endmodule




module FORWARD(DATA2, RESULT);
    
    //8bit buses for input and output
    input [7:0] DATA2;				
    output reg [7:0] RESULT;

    always @(DATA2)	
    #1//delay for Forward
    begin
      RESULT =  DATA2;
    end
endmodule

module ADD(DATA1, DATA2, RESULT);

    //8bit buses for input and outputs
    input [7:0] DATA1, DATA2;		
    output reg [7:0] RESULT;

    always @(DATA1,DATA2)	
    #2//delay for Add
    begin
      RESULT = DATA1 + DATA2;	
    end

endmodule

module AND(DATA1, DATA2, RESULT);
    
    //8bit buses for input and outputs
    input [7:0] DATA1, DATA2;		
    output reg [7:0] RESULT;


    always @(DATA1,DATA2)		
    #1//delay for And
    begin
      RESULT = DATA1 & DATA2;	
    end

endmodule

module OR(DATA1, DATA2, RESULT);	
    
    //8bit buses for input and output
    input [7:0] DATA1, DATA2;	
    output reg [7:0] RESULT;

    always @(DATA1,DATA2)		
    #1//delay or Or
    begin
    RESULT =  DATA1 | DATA2;		
    end

endmodule
