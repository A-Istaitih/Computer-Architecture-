/* 046267 Computer Architecture - HW #1                                 */
/* This file should hold your implementation of the predictor simulator */
/*Istaitih and Jawabra*/

#include <vector>
#include "bp_api.h"
#include <cmath>

using  namespace std;

typedef enum{ //local or shared
	not_using_shared =0, 
	using_share_lsb = 1, 
	using_share_mid =2
} savedShared;

enum state{ //biomodal states
	STRONGLY_NOT_TAKEN =0,
	WEAKLY_NOT_TAKEN =1,
	WEAKLY_TAKEN =2,
	STRONGLY_TAKEN =3
};
unsigned power(unsigned HistorySize)
{
    unsigned res = 1;
    for (unsigned i = 0; i < HistorySize; ++i) {
        res *= 2;
    }
    return res;
}

class BTB{
	public :
		unsigned m_btbSize;
		unsigned m_tagSize;
		unsigned m_historySize;
		bool m_isGlobalHist;
		bool m_isGlobalTable;
		savedShared m_Shared;
		state m_state;
		SIM_stats m_simStats;
		vector<uint32_t> m_tags;
		vector<uint32_t> m_destination;
		vector<bool> m_branchExists;
		vector<unsigned> m_history;
		vector<state> m_globalTable;
		vector<vector<state>> m_localTable;
};

BTB bp;
uint32_t bitXOR(uint32_t pc)
{
    uint32_t x=0;
    if(bp.m_isGlobalTable)
    {
        if(bp.m_Shared==using_share_lsb) // get the lowest bits from the pc
        {
            x=(pc>>2)% power(bp.m_historySize);
            return x;
        }
        if(bp.m_Shared==using_share_mid) //get the middle bits from the pc
        {
            x=(pc>>16)% power(bp.m_historySize);
            return x;
        }
    }
    return 0;

}

int ValidBTBParam(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState, int Shared){
	if(!(((btbSize <= 32) && (btbSize > 1) && (btbSize % 2 == 0))||(btbSize == 1))){
		return -1;
	}
	if(!(fsmState >= 0 && fsmState <= 3)){
		return -1;
	}
	if(!(tagSize <= 30-log2(btbSize) && fsmState >= 0)){
		return -1;
	}
	if(Shared <0 || Shared >2){
		return -1;
	}
	if(!(historySize >= 1 && historySize <= 8)){
		return -1;
	}
	return 0;
}

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){
	if(ValidBTBParam(btbSize,historySize,tagSize,fsmState,Shared) == -1){
		return -1;
	}	
	bp.m_btbSize = btbSize;
	bp.m_historySize = historySize;
	bp.m_tagSize = tagSize;
	bp.m_isGlobalHist = isGlobalHist;
	bp.m_isGlobalTable = isGlobalTable;
	bp.m_Shared = savedShared(Shared);
	bp.m_state = static_cast<state>(fsmState);
	bp.m_destination = vector<uint32_t>(btbSize,0);
	bp.m_tags = vector<uint32_t>(btbSize,0);
	bp.m_branchExists = vector<bool>(btbSize,false);
	bp.m_simStats = {0,0,0};

	if(isGlobalHist){
		bp.m_history = vector<unsigned>(1,0); //global history table so one entry
	}
	else{
		bp.m_history = vector<unsigned>(btbSize,0);
	}
	if(isGlobalTable){
		bp.m_globalTable = vector<state>(power(historySize),state(fsmState)); //initialize with fsmState
	}
	else{
		bp.m_localTable = vector<vector<state>>(btbSize,vector<state>(power(historySize),state(fsmState)));
	}
	return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst){
	uint32_t pcIndex = (pc >> 2) % bp.m_btbSize;
	uint32_t pcTag = ((pc >> 2)/bp.m_btbSize) % power(bp.m_tagSize);

	if(bp.m_branchExists[pcIndex] == false || bp.m_tags[pcIndex] != pcTag){ // not found in btb or tag mismatch
		*dst = pc+4;
		return false;
	}

	uint32_t xored = bitXOR(pc); //value to xor with history

	if(bp.m_isGlobalTable && bp.m_isGlobalHist){
		if(bp.m_globalTable[bp.m_history[0] ^ xored] == STRONGLY_TAKEN || bp.m_globalTable[bp.m_history[0] ^ xored] == WEAKLY_TAKEN){
			*dst = bp.m_destination[pcIndex]; //predicted taken
			return true;
		}
		else{
			*dst = pc+4; //predicted not taken  
			return false;
		}
	}

	if(bp.m_isGlobalHist){
		if(bp.m_localTable[pcIndex][bp.m_history[0]] == STRONGLY_TAKEN || bp.m_localTable[pcIndex][bp.m_history[0]] == WEAKLY_TAKEN){
			*dst = bp.m_destination[pcIndex]; //predicted taken
			return true;
		}
		else{
			*dst = pc+4; //predicted not taken
			return false;
		}
	}

	if(bp.m_isGlobalTable){
		if(bp.m_globalTable[bp.m_history[pcIndex] ^ xored] == STRONGLY_TAKEN || bp.m_globalTable[bp.m_history[pcIndex] ^ xored] == WEAKLY_TAKEN){
			*dst = bp.m_destination[pcIndex];
			return true;
		}
		else{
			*dst = pc+4;
			return false;
		}
	}

	if(bp.m_localTable[pcIndex][bp.m_history[pcIndex]] == STRONGLY_TAKEN || bp.m_localTable[pcIndex][bp.m_history[pcIndex]] == WEAKLY_TAKEN){
			*dst = bp.m_destination[pcIndex];
			return true;
	}
	*dst = pc+4;
	return false;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
// update the predictor state for each state
    bp.m_simStats.br_num++; // increment number of branches
    if((taken && targetPc!=pred_dst) || (!taken && pred_dst!=pc+4)) // mispredicted
    {
        bp.m_simStats.flush_num++; // increment number of flushes
    }

    uint32_t pcIndex= (pc>>2) %(bp.m_btbSize); // index in the btb
    uint32_t pcTag=((pc>>2)/(bp.m_btbSize))% power(bp.m_tagSize);
    bp.m_destination[pcIndex] = targetPc ; // update the destination
    bp.m_branchExists[pcIndex] = true; // mark that this entry is valid
    

    if(bp.m_tags[pcIndex] != pcTag){ // new entry , need to initialize
        if(!bp.m_isGlobalTable){
            bp.m_localTable[pcIndex] = vector<state>(power(bp.m_historySize),state(bp.m_state));
        }
        if(!bp.m_isGlobalHist){
            bp.m_history[pcIndex] = 0 ;
        }
        bp.m_tags[pcIndex] = pcTag ;

    }

    if((bp.m_isGlobalHist) && !(bp.m_isGlobalTable) ) // global history and local table
    {
        if(bp.m_localTable[pcIndex][bp.m_history[0]]!= STRONGLY_NOT_TAKEN && !taken )
        {
            int Decrement =(int) bp.m_localTable[pcIndex][bp.m_history[0]] ;
            Decrement -- ;
            bp.m_localTable[pcIndex][bp.m_history[0]] = (state)Decrement ;
        }
        if(bp.m_localTable[pcIndex][bp.m_history[0]]!= STRONGLY_TAKEN && taken )
        {
            int sum =(int) bp.m_localTable[pcIndex][bp.m_history[0]] ;
            sum ++ ;
            bp.m_localTable[pcIndex][bp.m_history[0]] = (state)sum ;
        }

        bp.m_history[0] = (bp.m_history[0]*2) % power(bp.m_historySize  )  + taken ; // update global history

    }

    if(!bp.m_isGlobalHist && !bp.m_isGlobalTable) // local history and local table
        {
        if (bp.m_localTable[pcIndex][bp.m_history[pcIndex]] != STRONGLY_NOT_TAKEN && !taken) {
            int Decrement = (int) bp.m_localTable[pcIndex][bp.m_history[pcIndex]];
            Decrement--;
            bp.m_localTable[pcIndex][bp.m_history[pcIndex]] = (state) Decrement;
        }

        if (bp.m_localTable[pcIndex][bp.m_history[pcIndex]] != STRONGLY_TAKEN && taken) {
            int sum = (int) bp.m_localTable[pcIndex][bp.m_history[pcIndex]];
            sum++;
            bp.m_localTable[pcIndex][bp.m_history[pcIndex]] = (state) sum;
        }

        bp.m_history[pcIndex] =(bp.m_history[pcIndex] * 2) % power(bp.m_historySize ) + taken;
        }
        uint32_t Xored= bitXOR(pc);
    if(bp.m_isGlobalHist && bp.m_isGlobalTable)
    {
      if(bp.m_globalTable[bp.m_history[0]^ Xored] != STRONGLY_NOT_TAKEN && !taken)
      {
          int Decrement =(int) bp.m_globalTable[bp.m_history[0] ^ Xored ] ;
          Decrement -- ;
          bp.m_globalTable[bp.m_history[0]^ Xored ] = (state)Decrement ;
      }
        if(bp.m_globalTable[bp.m_history[0]^ Xored] != STRONGLY_TAKEN && taken)
        {
            int sum =(int) bp.m_globalTable[bp.m_history[0] ^ Xored] ;
            sum ++ ;
            bp.m_globalTable[bp.m_history[0] ^ Xored ] = (state)sum ;
        }

        bp.m_history[0] = ((bp.m_history[0]*2) % power(bp.m_historySize ) ) + taken ;

    }

    if(!bp.m_isGlobalHist && bp.m_isGlobalTable)
    {

      if(bp.m_globalTable[bp.m_history[pcIndex] ^ Xored]!= STRONGLY_NOT_TAKEN && !taken )
        {
            int Decrement =(int) bp.m_globalTable[bp.m_history[pcIndex] ^ Xored] ;
            Decrement -- ;
            bp.m_globalTable[bp.m_history[pcIndex] ^ Xored] = (state)Decrement ;
        }
      if(bp.m_globalTable[bp.m_history[pcIndex] ^ Xored]!= STRONGLY_TAKEN && taken )
      {
          int sum =(int) bp.m_globalTable[bp.m_history[pcIndex] ^ Xored] ;
          sum ++ ;
          bp.m_globalTable[bp.m_history[pcIndex] ^ Xored] = (state)sum ;
      }

        bp.m_history[pcIndex] =(bp.m_history[pcIndex] * 2) % power(bp.m_historySize ) + taken;

    }

}

void BP_GetStats(SIM_stats *curStats){ 
    // calculate the size
    if(bp.m_isGlobalTable && bp.m_isGlobalHist)
    {
        bp.m_simStats.size=bp.m_btbSize * (bp.m_tagSize + 1 + 30);
        bp.m_simStats.size+= 2*power(bp.m_historySize) + bp.m_historySize;
    }
    else if(bp.m_isGlobalHist && !bp.m_isGlobalTable)
    {
        bp.m_simStats.size=bp.m_historySize;
        bp.m_simStats.size+= bp.m_btbSize* (bp.m_tagSize + 1 + 30 + 2*power(bp.m_historySize));
    }
    else if(!bp.m_isGlobalHist  && bp.m_isGlobalTable)
    {
        bp.m_simStats.size=2 * power(bp.m_historySize);
        bp.m_simStats.size+= bp.m_btbSize * (bp.m_tagSize + bp.m_historySize + 1 + 30);
    }
    else {
        bp.m_simStats.size = (2 * power(bp.m_historySize)) + 1 + 30 + bp.m_tagSize +
                                  bp.m_historySize;
        bp.m_simStats.size = bp.m_btbSize * bp.m_simStats.size;
    }
    // set the stats
    curStats->size = bp.m_simStats.size;
    curStats->flush_num= bp.m_simStats.flush_num;
    curStats->br_num=bp.m_simStats.br_num;
    return;
}


