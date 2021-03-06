/******************************************************************************
 This Source Code Form is subject to the terms of the
 Open Hardware Description License, v. 1.0. If a copy
 of the OHDL was not distributed with this file, You
 can obtain one at http://juliusbaxter.net/ohdl/ohdl.txt

 Description: Data cache implementation with write-back policy

 Copyright (C) 2012-2013
 Original implementation by:
    Stefan Kristiansson <stefan.kristiansson@saunalahti.fi>
    Stefan Wallentowitz <stefan.wallentowitz@tum.de>
 Modified by:
    Francesco Maio  <francima93@gmail.com>
	Giovanni Scotti <gio.scotti.94@gmail.com>

 ******************************************************************************/

`include "mor1kx-defines.v"

module mor1kx_dcache
  #(
    parameter OPTION_OPERAND_WIDTH = 32,          // bit
    parameter OPTION_DCACHE_BLOCK_WIDTH = 5,      // bit
    parameter OPTION_DCACHE_SET_WIDTH = 9,        // bit
    parameter OPTION_DCACHE_WAYS = 2,             // n-way set associative cache
    parameter OPTION_DCACHE_LIMIT_WIDTH = 32,
    parameter OPTION_DCACHE_SNOOP = "NONE"
    )
   (
    input 			      clk,                    // Clock signal
    input 			      rst,                    // Reset signal

    input 			      dc_dbus_err_i,
    input 			      dc_enable_i,
    input 			      dc_access_i,
	
	// Asserted when in the refill state
    output 			      refill_o,
	// Asserted when a miss occurred and refill is required
    output 			      refill_req_o,
	// Asserted when refill is completed
    output 			      refill_done_o,
	
	// ---------- DUMP VICTIM SIGNALS ----------
	
	// Data to be dumped in the LSU store buffer
	output reg [OPTION_OPERAND_WIDTH-1:0]     dump_dat_o,
	// Address of the data to be dumped
	output reg [OPTION_OPERAND_WIDTH-1:0]     dump_adr_o,
	// Control signal that tells the LSU to enter in the dump_victim state
	output                dump_req_o,
	// Asserted when dump is completed
	output                dump_done_o,
	// Asserted when an ack dump ("short" dump) takes place
	output                dump_mem_line_clean_o,
	// Drive store_buffer_write signal
	output reg            dump_write_valid_o,

    // ---------- CPU Interface ----------
	
    output 			      cpu_err_o,
	// Asserted when the cache puts data requested by LSU on cpu_dat_o
    output 			      cpu_ack_o,
    
    output                cache_hit_o,
    
	// The data that exits the cache
    output reg [OPTION_OPERAND_WIDTH-1:0] cpu_dat_o,
	// The data that enters the cache
    input [OPTION_OPERAND_WIDTH-1:0]      cpu_dat_i,
    input [OPTION_OPERAND_WIDTH-1:0]      cpu_adr_i,
	// In cpu_adr_match_i the 2 LSB are always 0
    input [OPTION_OPERAND_WIDTH-1:0]      cpu_adr_match_i,
	// Asserted when the LSU wants to perform a store or load operation and dc_access is true
    input 			      cpu_req_i,
	// Asserted when the LSU wants to perform a store
    input 			      cpu_we_i,
	// This is the byte selection
    input [3:0] 		  cpu_bsel_i,
    // Asserted when the LSU is ready to switch to the DC_REFILL state.
    // It is used to syncronize the transition of both the cache and the LSU to the REFILL state
    input 			      refill_allowed,

    // Address used in the REFILL state
    input [OPTION_OPERAND_WIDTH-1:0]  wradr_i,
    // Data used in the REFILL state
    input [OPTION_OPERAND_WIDTH-1:0]  wrdat_i,
    input 			      we_i,

    // Snoop address
    input [31:0] 		  snoop_adr_i,
    // Snoop event in this cycle
    input 			      snoop_valid_i,
    // Whether the snoop hit. If so, there will be no tag memory write
    // this cycle. The LSU may need to stall the pipeline.
    output 			      snoop_hit_o,


    // SPR interface
    input [15:0] 		  spr_bus_addr_i,
    input 			      spr_bus_we_i,
    input 			      spr_bus_stb_i,
    input [OPTION_OPERAND_WIDTH-1:0]  spr_bus_dat_i,

    output [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_o,
    output 			      spr_bus_ack_o
    );

   /*
   * A new state called "DUMP_VICTIM" is in charge of flushing a dirty cache line before
   * it is replaced in the REFILL state.
   */
	
   // States, hot encoded
   localparam IDLE		    = 6'b000001;
   localparam READ	     	= 6'b000010;
   localparam WRITE		    = 6'b000100;
   localparam REFILL		= 6'b001000;
   localparam INVALIDATE	= 6'b010000;
   localparam DUMP_VICTIM   = 6'b100000;

   // Address space in bytes for a way.
   localparam WAY_WIDTH = OPTION_DCACHE_BLOCK_WIDTH + OPTION_DCACHE_SET_WIDTH;
   /*
    * TAG MEMORY layout with the dirty bit implementation.
	* The index allows to identify a memory structure like the one provided below.
	* There is one dirty bit per each 32 bit block in the cache line.
	*
    * +--------------------------------------------------------------------------...
    * | LRU | wayN last dirty | ... | wayN first dirty | wayN valid | wayN tag | ...
    * +--------------------------------------------------------------------------...
    * ...--------------------------------------------------------------------+
    * ... | way0 last dirty | ... | way0 first dirty | way0 valid | way0 tag |
    * ...--------------------------------------------------------------------+
    */

   // Tag width in bits
   localparam TAG_WIDTH = (OPTION_DCACHE_LIMIT_WIDTH - WAY_WIDTH);
   // The number of dirty bits required. One per each 32 bit block in the cache line
   localparam QTY_DIRTY_BITS = (1 << OPTION_DCACHE_BLOCK_WIDTH - 2);
   // The tag memory contains entries with OPTION_DCACHE_WAYS parts of
   // TAGMEM_WAY_WIDTH bits each. They are composed (from right to left)
   // by the tag, the valid bit and the dirty bits.
   localparam TAGMEM_WAY_WIDTH = TAG_WIDTH + 1 + QTY_DIRTY_BITS;
   // Define the position of the MSB of the series of dirty bits contained in each entry
   // of the tag memory.
   localparam TAGMEM_WAY_DIRTY_MSB = TAGMEM_WAY_WIDTH - 1;
   // Define the position of the LSB of the series of dirty bits contained in each entry
   // of the tag memory.
   localparam TAGMEM_WAY_DIRTY_LSB = TAGMEM_WAY_WIDTH - QTY_DIRTY_BITS;
   // Position of the valid bit
   localparam TAGMEM_WAY_VALID = TAGMEM_WAY_WIDTH - QTY_DIRTY_BITS - 1;

   // Additionally, the tag memory entry contains an LRU value. The
   // width of this is 0 for OPTION_DCACHE_LIMIT_WIDTH==1
   localparam TAG_LRU_WIDTH = OPTION_DCACHE_WAYS*(OPTION_DCACHE_WAYS-1) >> 1;

   // We have signals for the LRU which are not used for one way
   // caches. To avoid signal width [-1:0] this generates [0:0]
   // vectors for them, which are removed automatically then.
   localparam TAG_LRU_WIDTH_BITS = (OPTION_DCACHE_WAYS >= 2) ? TAG_LRU_WIDTH : 1;

   // Compute the total sum of the entry elements
   localparam TAGMEM_WIDTH = TAGMEM_WAY_WIDTH * OPTION_DCACHE_WAYS + TAG_LRU_WIDTH;

   // For convenience we define the position of the LRU in the tag memory entries
   localparam TAG_LRU_MSB = TAGMEM_WIDTH - 1;
   localparam TAG_LRU_LSB = TAG_LRU_MSB - TAG_LRU_WIDTH + 1;

   // FSM state signals
   reg [5:0] 			  state;
   wire				      read;
   wire				      write;
   wire				      refill;
   wire                   dump_victim;

   reg [WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH] invalidate_adr;
   
   // The next address to be used in the refill state
   wire [31:0] 			  next_refill_adr;
   reg [31:0] 			  way_wr_dat;
   
   wire 			      refill_done;
   wire 			      refill_hit;
   
   // Dump signals
   wire                   dump_done;
   reg                    dump_done_r;
   wire                   dump_clearance;
   reg [WAY_WIDTH-3:0]    dump_adr;
   reg [WAY_WIDTH-3:0]    dump_adr_r;
   reg [TAG_WIDTH-1:0]    dump_tag;
   wire                   dump_mem_line_clean;
   
   /*
   * These are 8 bit variables, one bit per each possible 32-bit block in the cache block.
   * By default the cache block is made of 32 Bytes, therefore 8 32 bit block are contained into it.
   */
   reg [(1<<(OPTION_DCACHE_BLOCK_WIDTH-2))-1:0] refill_valid;
   reg [(1<<(OPTION_DCACHE_BLOCK_WIDTH-2))-1:0] refill_valid_r;
   
   reg [(1<<(OPTION_DCACHE_BLOCK_WIDTH-2))-1:0] dump_valid;
   
   wire				      invalidate;

   // The index we read and write from tag memory
   wire [OPTION_DCACHE_SET_WIDTH-1:0] tag_rindex;
   reg [OPTION_DCACHE_SET_WIDTH-1:0]  tag_windex;

   // The data from the tag memory
   // Given a set, this is the information about all the tags and lru
   wire [TAGMEM_WIDTH-1:0] 	          tag_dout;
   // This is the lru related to a set
   wire [TAG_LRU_WIDTH_BITS-1:0]      tag_lru_out;
   // For each way of a given set, this variable contains information about
   // the tag, the valid and the dirty bit
   wire [TAGMEM_WAY_WIDTH-1:0] 	      tag_way_out [OPTION_DCACHE_WAYS-1:0];

   // The data to the tag memory
   wire [TAGMEM_WIDTH-1:0] 	          tag_din;
   reg [TAG_LRU_WIDTH_BITS-1:0]       tag_lru_in;
   reg [TAGMEM_WAY_WIDTH-1:0] 	      tag_way_in [OPTION_DCACHE_WAYS-1:0];

   // This is a copy of the way. It is used in refill state
   reg [TAGMEM_WAY_WIDTH-1:0] 	      tag_way_save[OPTION_DCACHE_WAYS-1:0];

   // Whether to write to the tag memory in this cycle
   reg 				              tag_we;

   // This is the tag we need to write to the tag memory during refill
   wire [TAG_WIDTH-1:0] 	      tag_wtag;

   // This is the tag we check against
   wire [TAG_WIDTH-1:0] 	      tag_tag;

   // Access to the way memories
   // "-3" is due to the fact that the 2 lsb of the address are not needed (32 bit addressing).
   wire [WAY_WIDTH-3:0] 	          way_raddr[OPTION_DCACHE_WAYS-1:0];
   wire [WAY_WIDTH-3:0] 	          way_waddr[OPTION_DCACHE_WAYS-1:0];
   wire [OPTION_OPERAND_WIDTH-1:0]    way_din[OPTION_DCACHE_WAYS-1:0];
   wire [OPTION_OPERAND_WIDTH-1:0]    way_dout[OPTION_DCACHE_WAYS-1:0];
   reg [OPTION_DCACHE_WAYS-1:0]       way_we;

   // Does any way hit?
   wire 			      hit;
   // OPTION_DCACHE_WAYS number of bits. 1 if the way hits, 0 otherwise
   wire [OPTION_DCACHE_WAYS-1:0]      way_hit;
   // Asserted whenever a way is valid, but also dirty
   wire [OPTION_DCACHE_WAYS-1:0]      way_dirty;
   // Asserted whenever a way is valid, but also clean
   wire [OPTION_DCACHE_WAYS-1:0]      way_clean;

   // This is the least recently used value before access the memory.
   // Those are one hot encoded.
   wire [OPTION_DCACHE_WAYS-1:0]      lru;

   // Variable that stores the LRU value from lru variable
   reg [OPTION_DCACHE_WAYS-1:0]       tag_save_lru;

   // The access vector to update the LRU history is the way that has
   // a hit or is refilled. It is also one-hot encoded.
   reg [OPTION_DCACHE_WAYS-1:0]       access;

   // The current LRU history as read from tag memory and the update
   // value after we accessed it to write back to tag memory.
   wire [TAG_LRU_WIDTH_BITS-1:0]      current_lru_history;
   wire [TAG_LRU_WIDTH_BITS-1:0]      next_lru_history;

   // Intermediate signals to ease debugging
   wire [TAG_WIDTH-1:0]               check_way_tag [OPTION_DCACHE_WAYS-1:0];
   wire                               check_way_match [OPTION_DCACHE_WAYS-1:0];
   wire                               check_way_valid [OPTION_DCACHE_WAYS-1:0];
   wire [QTY_DIRTY_BITS-1:0]          check_way_dirty [OPTION_DCACHE_WAYS-1:0];
   
   // This is the offset of the currently dumped 32-bit block in the cache line
   wire [OPTION_DCACHE_BLOCK_WIDTH-2-1:0] dump_offset;

   reg 				                  write_pending;
   
   // The following signals are used in order to understand if the refill state
   // is accessed from the read state or the write state.
   reg                                from_read;
   reg                                from_write;
   reg                                from_refill;

   // Extract index to read from snooped address
   wire [OPTION_DCACHE_SET_WIDTH-1:0] snoop_index;
   
   assign snoop_index = snoop_adr_i[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH];

   // Register that is high one cycle after the actual snoop event to
   // drive the comparison
   reg 				                  snoop_check;
   // Register that stores the tag for one cycle
   reg [TAG_WIDTH-1:0] 		          snoop_tag;
   // Also store the index for one cycle, for the succeeding write access
   reg [OPTION_DCACHE_SET_WIDTH-1:0]  snoop_windex;

   // Snoop tag memory interface
   // Data out of tag memory
   wire [TAGMEM_WIDTH-1:0] 	          snoop_dout;
   // Each ways information in the tag memory
   wire [TAGMEM_WAY_WIDTH-1:0] 	      snoop_way_out [OPTION_DCACHE_WAYS-1:0];
   // Each ways tag in the tag memory
   wire [TAG_WIDTH-1:0] 	          snoop_check_way_tag [OPTION_DCACHE_WAYS-1:0];
   // Whether the tag matches the snoop tag
   wire                               snoop_check_way_match [OPTION_DCACHE_WAYS-1:0];
   // Whether the tag is valid
   wire                               snoop_check_way_valid [OPTION_DCACHE_WAYS-1:0];
   // Whether the way hits
   wire [OPTION_DCACHE_WAYS-1:0]      snoop_way_hit;
   // Whether any way hits
   wire 			                  snoop_hit;

   assign snoop_hit_o = (OPTION_DCACHE_SNOOP != "NONE") ? snoop_hit : 0;

   // Variable used in the for cycles to identify a way
   genvar 			      i;

   // Tells the LSU that the required data have just been sent to it or
   // that data have been written into the cache and the store operation is completed
   assign cpu_ack_o = (((read | refill) & hit & !write_pending | refill_hit) |
                        write & hit) & cpu_req_i & !snoop_hit;

   // Index to read a specific set
   assign tag_rindex = cpu_adr_i[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH];

   // Tag to compare with the ones in the cache
   assign tag_tag = cpu_adr_match_i[OPTION_DCACHE_LIMIT_WIDTH-1:WAY_WIDTH];
   
   // Tag to write to the tag memory during refill
   assign tag_wtag = wradr_i[OPTION_DCACHE_LIMIT_WIDTH-1:WAY_WIDTH];

   generate
      if (OPTION_DCACHE_WAYS >= 2) begin
         // Multiplex the LRU history from and to tag memory
         assign current_lru_history = tag_dout[TAG_LRU_MSB:TAG_LRU_LSB];
		 // Add information about lru to be stored in the tag memory
         assign tag_din[TAG_LRU_MSB:TAG_LRU_LSB] = tag_lru_in;
         assign tag_lru_out = tag_dout[TAG_LRU_MSB:TAG_LRU_LSB];
      end

	  for (i = 0; i < OPTION_DCACHE_WAYS; i=i+1) begin : ways
	  
		 // Address used to access a way. It is taken from the incoming address
		 // If in the DUMP_VICTIM state, the read address is dump_adr.
		 // This is why we need to increment the address in each clock cycle
	     assign way_raddr[i] = (dump_victim) ? dump_adr : cpu_adr_i[WAY_WIDTH-1:2];
		 // We can write into the way memory only in the write state and in the refill state
	     assign way_waddr[i] = write ? cpu_adr_match_i[WAY_WIDTH-1:2] : wradr_i[WAY_WIDTH-1:2];
	     // Data to copy into the way memory
	     assign way_din[i] = way_wr_dat;

	     // Compare stored tag with incoming tag and check valid bit.
		 // It contains the tag of each way
         assign check_way_tag[i] = tag_way_out[i][TAG_WIDTH-1:0];
		 // Compare the incoming tag with the tag in each way
         assign check_way_match[i] = (check_way_tag[i] == tag_tag);
		 // It stores the value of the valid bit of each way
         assign check_way_valid[i] = tag_way_out[i][TAGMEM_WAY_VALID];
		 // It stores the value of the dirty bits of each way
		 assign check_way_dirty[i] = tag_way_out[i][TAGMEM_WAY_DIRTY_MSB:TAGMEM_WAY_DIRTY_LSB];
		 
		 // Did a hit occur?
         assign way_hit[i] = check_way_valid[i] & check_way_match[i];	 
		 
		 /*
		 * Asserted whenever a way is valid, but also dirty
		 * This is used in the read/write states in order to understand if the way must be evicted.
		 * If the way is dirty, but it is not valid, it must not be dumped.
		 * The way to be dumped is given by lru.
		 * Please notice that a way is flushed if at least one 32 bit block is dirty.
		 */
		 assign way_dirty[i] = check_way_valid[i] & |(check_way_dirty[i]);

         // Asserted when a way is valid, but not dirty. This is done in order to figure out
         // when the dump_mem_line_clean (memory acknowledgement) signal is required
         assign way_clean[i] = check_way_valid[i] & !(|(check_way_dirty[i]));
		 
         // Multiplex the way entries in the tag memory and concatenate the tags and flags
		 // of the different ways
         assign tag_din[(i+1)*TAGMEM_WAY_WIDTH-1:i*TAGMEM_WAY_WIDTH] = tag_way_in[i];
		 // tag_way_out contains the dirty bit, the valid bit and the tag
         assign tag_way_out[i] = tag_dout[(i+1)*TAGMEM_WAY_WIDTH-1:i*TAGMEM_WAY_WIDTH];

	     if (OPTION_DCACHE_SNOOP != "NONE") begin
	        // The same for the snoop tag memory
            assign snoop_way_out[i] = snoop_dout[(i+1)*TAGMEM_WAY_WIDTH-1:i*TAGMEM_WAY_WIDTH];

	        assign snoop_check_way_tag[i] = snoop_way_out[i][TAG_WIDTH-1:0];
	        assign snoop_check_way_match[i] = (snoop_check_way_tag[i] == snoop_tag);
	        assign snoop_check_way_valid[i] = snoop_way_out[i][TAGMEM_WAY_VALID];

	        assign snoop_way_hit[i] = snoop_check_way_valid[i] & snoop_check_way_match[i];
	     end
      end
   endgenerate

   // hit is true if there is at least a hit in one way
   assign hit = |way_hit;
   assign cache_hit_o = hit;

   assign snoop_hit = (OPTION_DCACHE_SNOOP != "NONE") &
		      |snoop_way_hit & snoop_check;

   // This code block is in charge of returning the data to the LSU
   integer w0;
   always @(*) begin
      cpu_dat_o = {OPTION_OPERAND_WIDTH{1'bx}};
	  dump_dat_o = {OPTION_OPERAND_WIDTH{1'bx}};

      // Put correct way on the data port
      for (w0 = 0; w0 < OPTION_DCACHE_WAYS; w0 = w0 + 1) begin
         if (way_hit[w0] | (refill_hit & tag_save_lru[w0])) begin
		 
		 
		    /*************************
			* There is an OPTIMIZATION:
			*
			* The first 32 refilled bits are immediately sent to the LSU
			* since they are the data required by it. Then, the refill procedure
			* continues until its termination.
			*
			*************************/
		
			// way_dout is the 32 bit data contained in the hit way
            cpu_dat_o = way_dout[w0];
         end
		 if (lru[w0]) begin
		    dump_dat_o = way_dout[w0];
		 end
      end
   end

   
   // Compute the next address to get the following 32 bit of data.
   // It is used when refilling
   assign next_refill_adr = (OPTION_DCACHE_BLOCK_WIDTH == 5) ?
			    {wradr_i[31:5], wradr_i[4:0] + 5'd4} : // 32 byte
			    {wradr_i[31:4], wradr_i[3:0] + 4'd4};  // 16 byte

   // Tell the lsu whether the refill is done or not
   assign refill_done_o = refill_done;
   // refill_done is equal to the value of the bit of refill_valid that corresponds to the
   // next 32 bit to be refilled in the cache block. True when the entire block has been refilled.
   // It's something like a circular mechanism
   assign refill_done = refill_valid[next_refill_adr[OPTION_DCACHE_BLOCK_WIDTH-1:2]];
   
   // True when the missing 32 bit group (the one that caused the cache miss) of the cache block has been replaced.
   // The address must be incremented by 4 in order to load another 32 bit group in the block.
   // Pay attention that the "& from_read" condition has been added in order to prevent the refill_hit
   // signal from going high when the refill is caused by a write
   assign refill_hit = (refill_valid_r[cpu_adr_match_i[OPTION_DCACHE_BLOCK_WIDTH-1:2]] &
		       cpu_adr_match_i[OPTION_DCACHE_LIMIT_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH] ==
		       wradr_i[OPTION_DCACHE_LIMIT_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH] &
		       refill & !write_pending) & from_read;

   assign refill = (state == REFILL);
   assign read = (state == READ);
   assign write = (state == WRITE);
   assign dump_victim = (state == DUMP_VICTIM);

   assign refill_o = refill;
   
   // Refill is required if in the read/write state a miss occurs and the cache line is not dirty.
   // Otherwise it is required after the dump of the line
   assign refill_req_o = (dump_victim | write | read) & cpu_req_i & !hit & dump_done & refill_allowed & !from_refill | refill;
  
   // dump_req_o signal is in charge of controlling the transition of the LSU to the DC_DUMP_VICTIM state
   assign dump_req_o = (read | write) & cpu_req_i & !hit & (dump_clearance | dump_mem_line_clean) | dump_victim;
   
   // Tell the lsu whether the dump is done or not
   assign dump_done_o = dump_done;
   
   // Asserted if dump took place and it's done or if there is no need it happens
   // It also takes into account the case of "short" dump to send an ack to memory
   assign dump_done = (dump_mem_line_clean) ? |(dump_valid) :
                                       &(dump_valid) | !dump_clearance;
   
   // Asserted if the dump procedure can take place
   assign dump_clearance = |(lru & way_dirty);

   // When there is a miss, but none of the bits of the line is dirty,
   // the whole architecture (in particular the memory) must be informed that
   // the data cache is not anymore the owner of that line, since it is going to be replaced.
   // All this happens with a "short" dump in which only the first 32 bits of the cache line
   // are sent to the store buffer in the LSU.
   assign dump_mem_line_clean = |(lru & way_clean);
   
   // Acknowledgement that is sent to the memory when the cache line is clean, but needs to be evicted
   assign dump_mem_line_clean_o = dump_victim & dump_mem_line_clean;
   
   assign dump_offset = dump_adr_r[OPTION_DCACHE_BLOCK_WIDTH-2-1:0];
   
   /*
    * SPR bus interface
    */

   // The SPR interface is used to invalidate the cache blocks. When
   // an invalidation is started, the respective entry in the tag
   // memory is cleared. When another transfer is in progress, the
   // handling is delayed until it is possible to serve it.
   //
   // The invalidation is acknowledged to the SPR bus, but the cycle
   // is terminated by the core. We therefore need to hold the
   // invalidate acknowledgement. Meanwhile we continuously write the
   // tag memory which is no problem.

   // Net that signals an acknowledgement
   reg invalidate_ack;

   // An invalidate request is either a block flush or a block invalidate
   assign invalidate = spr_bus_stb_i & spr_bus_we_i &
		       (spr_bus_addr_i == `OR1K_SPR_DCBFR_ADDR |
			spr_bus_addr_i == `OR1K_SPR_DCBIR_ADDR);

   // Acknowledge to the SPR bus.
   assign spr_bus_ack_o = invalidate_ack;

   /*
    * Cache FSM
    * Starts in IDLE.
    * State changes between READ and WRITE happens cpu_we_i is asserted or not.
    * cpu_we_i is in sync with cpu_adr_i, so that means that it's the
    * *upcoming* write that it is indicating. It only toggles for one cycle,
    * so if we are busy doing something else when this signal comes
    * (i.e. refilling) we assert the write_pending signal.
    * cpu_req_i is in sync with cpu_adr_match_i, so it can be used to
    * determined if a cache hit should cause a refill or if a write should
    * really be executed.
    */
   integer w1;
   always @(posedge clk `OR_ASYNC_RST) begin
      if (rst) begin
	     state <= IDLE;
	     write_pending <= 0;
	     from_refill <= 0;
	     dump_valid <= 0;
	     dump_adr <= 0;
	     dump_adr_r <= 0;
      end else if(dc_dbus_err_i) begin
	     state <= IDLE;
	     write_pending <= 0;
      end else begin
	     if (cpu_we_i)
	        write_pending <= 1;
	     else if (!cpu_req_i)
	        write_pending <= 0;

	     refill_valid_r <= refill_valid;

		 dump_adr_r <= dump_adr;
		 dump_done_r <= dump_done;

	     if (snoop_valid_i) begin
	     //
	     // If there is a snoop event, we need to store this information.
		 // This happens independently of whether we have a snoop tag memory or not.
	     //
	        snoop_check <= 1;
	        snoop_windex <= snoop_index;
	        snoop_tag <= snoop_adr_i[OPTION_DCACHE_LIMIT_WIDTH-1:WAY_WIDTH];
	     end else begin
	        snoop_check <= 0;
	     end

	  case (state)
	     IDLE: begin
	        if (invalidate) begin
		    // If there is an invalidation request
		    // Store address in invalidate_adr that is muxed to the tag
		    // memory write address
		       invalidate_adr <= spr_bus_dat_i[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH];

		    // Change to invalidate state that actually accesses the tag memory
		       state <= INVALIDATE;
	        end else if (cpu_we_i | write_pending)
		       state <= WRITE;
	        else if (cpu_req_i)
		       state <= READ;
	     end

		 READ: begin
	        if (dc_access_i | cpu_we_i & dc_enable_i) begin
		       if (!hit & cpu_req_i & !write_pending) begin
		          // Initialization of the refill state
		          refill_valid <= 0;
		          refill_valid_r <= 0;

		          // Store the LRU information for correct replacement on refill.
		          // Always one when only one way in the cache
                  tag_save_lru <= (OPTION_DCACHE_WAYS==1) | lru;

				  // Save all the ways in order to restore them at the end
				  // of the replacement of the target cache line
		          for (w1 = 0; w1 < OPTION_DCACHE_WAYS; w1 = w1 + 1) begin
		             tag_way_save[w1] <= tag_way_out[w1];
		          end
				  
				  // Highlight the fact that we come from the READ state
				  from_read <= 1;
				  from_write <= 0;
				  
				  // Dump must happen when the cache line is valid and dirty or
				  // valid and clean. In the latter case, an acknowledgement to the memory is sent
				  if (dump_clearance | dump_mem_line_clean) begin
				     // Initialization of the dump state
				     dump_valid <= 0;
				     // Take the address of the set and concatenate it with 0
                     // This is the address to increment by 1 to carry out the dump.
                     dump_adr <= {cpu_adr_match_i[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH], {{OPTION_DCACHE_BLOCK_WIDTH-2}{1'b0}}};
                     // Initialize the tag for the dump process and building the dump address
                     // check_way_tag[lru] already contains the tag of the way that will be dumped
                     for (w1 = 0; w1 < OPTION_DCACHE_WAYS; w1 = w1 + 1) begin
                        if (lru[w1]) begin
                           dump_tag <= check_way_tag[w1];
                        end
                     end
                     
				     state <= DUMP_VICTIM;
				  end else if (refill_allowed) begin
				     state <= REFILL;
				  end   
		       end else if (cpu_we_i | write_pending) begin
		          state <= WRITE;
		       end else if (invalidate) begin
		          state <= IDLE;
		       end
	        end else if (!dc_enable_i | invalidate) begin
			   state <= IDLE;
	        end
	     end
		 
		 DUMP_VICTIM: begin
		    // Check if dirty bit == 1 (it remains asserted for all the duration of the dump)
			// The dirty bit will be reset at the end of the refill state
			if (dump_done & refill_allowed) begin
			   from_refill <= 0;
			   state <= REFILL;
			end else if ((dump_clearance | dump_mem_line_clean) & !dump_done) begin
			   dump_valid[dump_adr[OPTION_DCACHE_BLOCK_WIDTH-1-2:0]] <= 1;
			   // Increase the address by 1 because the two LSB have been cut off
			   dump_adr <= dump_adr + 1;   
			end
		 end

	     REFILL: begin
	        if (we_i) begin
		       refill_valid[wradr_i[OPTION_DCACHE_BLOCK_WIDTH-1:2]] <= 1;

		       if (refill_done & from_read) begin
		          state <= IDLE;
		       // If refill is done and we came from write, then return to write state
		       // in order to carry out the writing of the target cache line
			   end else if (refill_done & from_write) begin
			      from_refill <= 1;
			      state <= WRITE;
			   end
	        end
	        // Abort refill on snoop-hit
	        // TODO: only abort on snoop-hits to refill address
	        if (snoop_hit) begin
		       refill_valid <= 0;
		       refill_valid_r <= 0;
		       state <= IDLE;
	        end
			
	     end

	     WRITE: begin
		    if (!hit & cpu_req_i & !from_refill) begin
			   // Initialization of the refill state
		       refill_valid <= 0;
		       refill_valid_r <= 0;

		       // Store the LRU information for correct replacement
               // on refill. Always one when only one way in the cache.
               tag_save_lru <= (OPTION_DCACHE_WAYS==1) | lru;

			   // Save all the ways in order to restore them at the end of the replacement
		       for (w1 = 0; w1 < OPTION_DCACHE_WAYS; w1 = w1 + 1) begin
		          tag_way_save[w1] <= tag_way_out[w1];
	           end
			   
			   // Highlight the fact that we come from the WRITE state
               from_read <= 0;
               from_write <= 1;
			   
			   // Dump must happen when the cache line is valid and dirty or
               // valid and clean. In the latter case, an acknowledgement to the memory is sent
			   if (dump_clearance | dump_mem_line_clean) begin
			      // Initialization of the dump state
                  dump_valid <= 0;
                  // Take the address of the set and concatenate it with 0
                  // This is the address to increment by 1 to carry out the dump.
                  dump_adr <= {cpu_adr_match_i[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH], {{OPTION_DCACHE_BLOCK_WIDTH-2}{1'b0}}};
                  // Initialize the tag for the dump process and building the dump address
                  // check_way_tag[lru] already contains the tag of the way that will be dumped
                  for (w1 = 0; w1 < OPTION_DCACHE_WAYS; w1 = w1 + 1) begin
                     if (lru[w1]) begin
                        dump_tag <= check_way_tag[w1];
                     end
                  end
			   
                  state <= DUMP_VICTIM;
               end else if (refill_allowed) begin
                  from_refill <= 0;
                  state <= REFILL;
               end
            
            // It is useful to return to the READ state because a subsequent load operation can happen
	        end else if ((!dc_access_i | !cpu_req_i | !cpu_we_i) & !snoop_hit & hit) begin
	           from_refill <= 0;
		       write_pending <= 0;
		       state <= READ;
	        end
	     end

	     INVALIDATE: begin
	        if (invalidate) begin
		    // Store address in invalidate_adr that is muxed to the tag
		    // memory write address
		       invalidate_adr <= spr_bus_dat_i[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH];

		       state <= INVALIDATE;
	        end else begin
		       state <= IDLE;
	        end
	     end

	     default:
	        state <= IDLE;
	  endcase
      end
   end

   // This is the combinational part of the state machine that interfaces the tag and way memories
   integer w2;
   always @(*) begin
      // Default is to keep data, don't write and don't access
      tag_lru_in = tag_lru_out;
      for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
         tag_way_in[w2] = tag_way_out[w2];
      end

	  // Write access to the tag memory
      tag_we = 1'b0;
	  // Write access to the way memory. The default is 0: nothing can be accessed
      way_we = {(OPTION_DCACHE_WAYS){1'b0}};
      // Access to the LRU module
      access = {(OPTION_DCACHE_WAYS){1'b0}};

      way_wr_dat = wrdat_i;

      // The default is (of course) not to acknowledge the invalidate
      invalidate_ack = 1'b0;
      
      // By default, do not write anything into the store buffer
      dump_write_valid_o = 0;

      if (snoop_hit) begin
	     // This is the write access
	     tag_we = 1'b1;
	     tag_windex = snoop_windex;
	     for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
	        if (snoop_way_hit[w2]) begin
	           tag_way_in[w2] = 0;
	        end else begin
	           tag_way_in[w2] = snoop_way_out[w2];
	        end
	     end
      end else begin
	     //
	     // The tag mem is written during reads and writes to write
	     // the lru info and  during refill and invalidate.
	     //
	     tag_windex = read | write ?
		      cpu_adr_match_i[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH] :
		      (state == INVALIDATE) ? invalidate_adr :
		      wradr_i[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH];

	  case (state)
	     IDLE: begin
	      /*
	      * When idle we can always acknowledge the invalidate as it
	      * has the highest priority in handling. When something is
	      * changed on the state machine handling above this needs
	      * to be changed.
	      */
	        invalidate_ack = 1'b1;
	     end

	     READ: begin
	        if (hit) begin
		       /*
		       * We got a hit. The LRU module gets the access
		       * information to be aware of the most recently accessed way.
		       * Depending on this we update the LRU history in the tag.
		       */
		       access = way_hit;

		       // This is the updated LRU history after hit
		       tag_lru_in = next_lru_history;
               // Enable tag memory write to update the LRU information
		       tag_we = 1'b1;
	        end
	     end

	     WRITE: begin
	        way_wr_dat = cpu_dat_i;
	        if (hit & cpu_req_i) begin
		    /* 
			* Mux cache output with write data.
			* It allows to write the received byte only leaving
			* the other data untouched.
			*/
		       if (!cpu_bsel_i[3])
		          way_wr_dat[31:24] = cpu_dat_o[31:24];
		       if (!cpu_bsel_i[2])
		          way_wr_dat[23:16] = cpu_dat_o[23:16];
		       if (!cpu_bsel_i[1])
		          way_wr_dat[15:8] = cpu_dat_o[15:8];
		       if (!cpu_bsel_i[0])
		          way_wr_dat[7:0] = cpu_dat_o[7:0];

			   // Select the way where to write
	           way_we = way_hit;
               // Update the LRU history
	           tag_lru_in = next_lru_history;
               // Enable tag memory write to update the LRU information
		       tag_we = 1'b1;	 
		       
		       // Set the dirty bit to 1 after writing
		       for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
		          if (tag_save_lru[w2]) begin
		             tag_way_in[w2][TAGMEM_WAY_DIRTY_LSB + cpu_adr_match_i[OPTION_DCACHE_BLOCK_WIDTH-1:2]] = 1'b1;
		          end
		       end   
	        end
	     end

         DUMP_VICTIM: begin
            // The two LSB are always 0 because we access 32 bit at a time
            dump_adr_o = {dump_tag,dump_adr_r,2'b00};
            
            // Drive store_buffer_write signal in order to save only dirty 32-bit blocks
            for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
               if (lru[w2]) begin
                  if(!dump_mem_line_clean) begin
                     // Dirty bit & dump_valid with respect to a 32-bit block must be asserted
                     dump_write_valid_o = check_way_dirty[w2][dump_offset] & dump_valid[dump_offset] & !dump_done_r;
                  end else begin
                     // If we are sending an acknowledgment to memory
                     dump_write_valid_o = |(dump_valid) & !dump_done_r;
                  end
               end
            end
         end

	     REFILL: begin
	        // we_i is the ack coming from the bus
	        if (we_i) begin
		       // Write the data to the way that is replaced (which is the LRU)
		       way_we = tag_save_lru;

		       // Access pattern
		       access = tag_save_lru;

		       // Invalidate the way on the first write
		       if (refill_valid == 0) begin
		          for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
                     if (tag_save_lru[w2]) begin
			            tag_way_in[w2][TAGMEM_WAY_VALID] = 1'b0;
                     end
                  end

		          tag_we = 1'b1;
		       end

		       /*
		       * After refill, update the tag memory entry of the
		       * filled way with the LRU history, the tag and set
		       * valid to 1.
		       */
		       if (refill_done) begin
		          for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
		             tag_way_in[w2] = tag_way_save[w2];
                        if (tag_save_lru[w2]) begin
					       // Set the dirty bits to 0, the valid bit to one and insert the new tag
						   tag_way_in[w2] = {{{QTY_DIRTY_BITS}{1'b0}}, 1'b1, tag_wtag};
                        end
                  end
                  tag_lru_in = next_lru_history;

		          tag_we = 1'b1;
		       end
	        end
	     end

	     INVALIDATE: begin
	        invalidate_ack = 1'b1;

	        // Lazy invalidation, invalidate everything that matches tag address
            tag_lru_in = 0;
            for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
		       tag_way_in[w2] = 0;
            end

	        tag_we = 1'b1;
	      end

	     default: begin
	     end
	  endcase
      end
   end

   generate
      for (i = 0; i < OPTION_DCACHE_WAYS; i=i+1) begin : way_memories
	     mor1kx_simple_dpram_sclk
	       #(
		 .ADDR_WIDTH(WAY_WIDTH-2),
		 .DATA_WIDTH(OPTION_OPERAND_WIDTH),
		 .ENABLE_BYPASS(1)
		 )
	     way_data_ram
	       (
		   // Outputs
		   .dout		(way_dout[i]),
		   // Inputs
		   .clk			(clk),
		   .raddr		(way_raddr[i][WAY_WIDTH-3:0]),
		   .re			(1'b1),
		   .waddr		(way_waddr[i][WAY_WIDTH-3:0]),
		   .we		    (way_we[i]),
		   .din			(way_din[i][31:0]));

      end

      if (OPTION_DCACHE_WAYS >= 2) begin : gen_u_lru
         mor1kx_cache_lru
           #(.NUMWAYS(OPTION_DCACHE_WAYS))
         u_lru(
	       // Outputs
	       .update			(next_lru_history),	     // Templated
	       .lru_pre			(lru),			         // Templated
	       .lru_post		(),			             // Templated
	       // Inputs
	       .current			(current_lru_history),	 // Templated
	       .access			(access));		         // Templated
      end // if (OPTION_DCACHE_WAYS >= 2)
   endgenerate

   mor1kx_simple_dpram_sclk
     #(
       .ADDR_WIDTH(OPTION_DCACHE_SET_WIDTH),
       .DATA_WIDTH(TAGMEM_WIDTH),
       .ENABLE_BYPASS(OPTION_DCACHE_SNOOP != "NONE")
     )
   tag_ram
     (
      // Outputs
      .dout				(tag_dout[TAGMEM_WIDTH-1:0]),
      // Inputs
      .clk				(clk),
      .raddr			(tag_rindex),
      .re				(1'b1),
      .waddr			(tag_windex),
      .we				(tag_we),
      .din				(tag_din));

   generate
      if (OPTION_DCACHE_SNOOP != "NONE") begin
         mor1kx_simple_dpram_sclk
         #(
           .ADDR_WIDTH(OPTION_DCACHE_SET_WIDTH),
           .DATA_WIDTH(TAGMEM_WIDTH),
           .ENABLE_BYPASS(1)
          )
         snoop_tag_ram
          (
          // Outputs
          .dout			(snoop_dout[TAGMEM_WIDTH-1:0]),
          // Inputs
          .clk			(clk),
          .raddr		(snoop_index),
          .re			(1'b1),
          .waddr		(tag_windex),
          .we			(tag_we),
          .din			(tag_din));
      end
   endgenerate

endmodule
