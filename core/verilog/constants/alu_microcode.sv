`ifndef __ALU_MICROCODE_SV__
`define __ALU_MICROCODE_SV__

/*
    Microcode encoding:
        [8]: Is BEQ / BNE {Outputs 010 / 011}
        [7]: Is BGE / BGEU / BNE instead of BLT / BLTU / BEQ {Outputs 010 / 011}
        [6]: Is SLTU / BLTU instead of SLT / BLT {outputs 010 / 011}
        [5]: Is ADD (active low) {outputs 000 / 010 / 011}
        [4:3]: Shifter operation (00=SLL, 01=SRL, 11=SRA) {outputs 001 / 101}
        [2:0]: Output select (000=ADD/SUB, 001=SLL, 010=SLT/B*, 011=SLTU, 100=XOR, 101=SRL/SRA, 110=OR, 111=AND)
*/

// Microcodes listed alongside their sensitivity masks

`define ALU_MICROCODE_MASK_SUM      {4'b0001, 2'b00, 3'b111}
`define ALU_MICROCODE_ADD           {4'b0000, 2'b00, 3'b000}
`define ALU_MICROCODE_SUB           {4'b0001, 2'b00, 3'b000}

`define ALU_MICROCODE_MASK_SHIFT    {4'b0000, 2'b11, 3'b111}
`define ALU_MICROCODE_SLL           {4'b0000, 2'b00, 3'b001}
`define ALU_MICROCODE_SRL           {4'b0000, 2'b01, 3'b101}
`define ALU_MICROCODE_SRA           {4'b0000, 2'b11, 3'b101}

`define ALU_MICROCODE_MASK_BRANCH   {4'b1111, 2'b00, 3'b111}
`define ALU_MICROCODE_SLT           {4'b0001, 2'b00, 3'b010}
`define ALU_MICROCODE_SLTU          {4'b0011, 2'b00, 3'b011}
`define ALU_MICROCODE_BEQ           {4'b1001, 2'b00, 3'b010}
`define ALU_MICROCODE_BNE           {4'b1101, 2'b00, 3'b010}
`define ALU_MICROCODE_BLT           {4'b0001, 2'b00, 3'b010}
`define ALU_MICROCODE_BGE           {4'b0101, 2'b00, 3'b010}
`define ALU_MICROCODE_BLTU          {4'b0011, 2'b00, 3'b010}
`define ALU_MICROCODE_BGEU          {4'b0111, 2'b00, 3'b010}

`define ALU_MICROCODE_MASK_LOGIC    {4'b0000, 2'b00, 3'b111}
`define ALU_MICROCODE_XOR           {4'b0000, 2'b00, 3'b100}
`define ALU_MICROCODE_OR            {4'b0000, 2'b00, 3'b110}
`define ALU_MICROCODE_AND           {4'b0000, 2'b00, 3'b111}

`endif
