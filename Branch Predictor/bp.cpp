/* 046267 Computer Architecture - HW #1                                 */
/* This file should hold your implementation of the predictor simulator */

#include <vector>
#include "bp_api.h"
#include <cmath>

using  namespace std;

typedef enum{
	not_using_shared =0, 
	using_share_lsb = 1, 
	using_share_mid =2
} savedShared;

enum state{
	STRONGLY_NOT_TAKEN =0,
	WEAKLY_NOT_TAKEN =1,
	WEAKLY_TAKEN =2,
	STRONGLY_TAKEN =3
};
unsigned power(unsigned exp) {
    unsigned long long result = 1;
	unsigned base = 2;
    while (exp > 0) {
        if (exp)            
            result *= base;
        base *= base;        
        exp -= 1;             
    }
    return result;
}

class BTB{
	public :
		unsigned m_btbSize;
		unsigned m_tagSize;
		unsigned m_historySize;
		bool m_isGlobalHist;
		bool m_isGlobalTable;
		int m_Shared;
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
uint32_t bitWiseXOR(uint32_t pc){
	uint32_t res = 0;

	if(bp.m_isGlobalTable){
		if(bp.m_Shared == using_share_lsb){
			res = (pc >> power(1))%power(bp.m_historySize);
			return res;
		}
		if(bp.m_Shared == using_share_mid){
			res = (pc >> power(4))%power(bp.m_historySize);
			return res;
		}	
	}
	return 0;
}
int ValidBTBParam(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState, int Shared){
	if(!(((btbSize <= 32) && (btbSize > 1) && (btbSize % 2 == 0))||(btbSize == 1))){
		return -1;
	}
	if(fsmState >3 || fsmState <0){
		return -1;
	}
	if(tagSize >30-log2(btbSize) || fsmState <0){
		return -1;
	}
	if(Shared <0 || Shared >2){
		return -1;
	}
	if(historySize <1 || historySize >8){
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
	bp.m_Shared = Shared;
	bp.m_state = static_cast<state>(fsmState);
	bp.m_destination = vector<uint32_t>(btbSize,0);
	bp.m_tags = vector<uint32_t>(btbSize,0);
	bp.m_branchExists = vector<bool>(btbSize,false);
	bp.m_simStats = {0,0,0};

	if(isGlobalHist){
		bp.m_history = vector<unsigned>(historySize,0);
	}
	else{
		bp.m_history = vector<unsigned>(btbSize,0);
	}
	if(isGlobalTable){
		bp.m_globalTable = vector<state>(power(historySize),state(fsmState));
	}
	else{
		bp.m_localTable = vector<vector<state>>(btbSize,vector<state>(power(historySize),state(fsmState)));
	}
	return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst){
	uint32_t pcIndex = (pc >> 2) % bp.m_btbSize;
	uint32_t pcTag = ((pc >> 2)/bp.m_btbSize) % power(bp.m_tagSize);

	if(bp.m_branchExists[pcIndex] == false || bp.m_tags[pcIndex] != pcTag){
		*dst = pc+4;
		return false;
	}

	uint32_t xored = bitWiseXOR(pc);

	if(bp.m_isGlobalTable && bp.m_isGlobalHist){
		if(bp.m_globalTable[bp.m_history[0] ^ xored] == STRONGLY_TAKEN || bp.m_globalTable[bp.m_history[0] ^ xored] == WEAKLY_TAKEN){
			*dst = bp.m_destination[pcIndex];
			return true;
		}
		else{
			*dst = pc+4;
			return false;
		}
	}

	if(bp.m_isGlobalHist){
		if(bp.m_localTable[pcIndex][bp.m_history[0] ^ xored] == STRONGLY_TAKEN || bp.m_localTable[pcIndex][bp.m_history[0] ^ xored] == WEAKLY_TAKEN){
			*dst = bp.m_destination[pcIndex];
			return true;
		}
		else{
			*dst = pc+4;
			return false;
		}
	}

	if(bp.m_isGlobalTable){
		if(bp.m_globalTable[bp.m_history[0] ^ xored] == STRONGLY_TAKEN || bp.m_globalTable[bp.m_history[0] ^ xored] == WEAKLY_TAKEN){
			*dst = bp.m_destination[pcIndex];
			return true;
		}
		else{
			*dst = pc+4;
			return false;
		}
	}

	if(bp.m_localTable[pcIndex][bp.m_history[0] ^ xored] == STRONGLY_TAKEN || bp.m_localTable[pcIndex][bp.m_history[0] ^ xored] == WEAKLY_TAKEN){
			*dst = bp.m_destination[pcIndex];
			return true;
	}
	*dst = pc+4;
	return false;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
    bp.m_simStats.br_num++;
    if(taken && targetPc!=pred_dst)
    {
        bp.m_simStats.flush_num++;
    }
    if(!taken && pred_dst!=pc+4)
    {
        bp.m_simStats.flush_num++;
    }

    // update now
    uint32_t pcIndex= (pc>>2) %(bp.m_btbSize);
    uint32_t pcTag=((pc>>2)/(bp.m_btbSize))% power(bp.m_tagSize);
    bp.m_branchExists[pcIndex] = true;
    bp.m_destination[pcIndex] = targetPc ;

   // check if we need to update the history or the table to default 	
    if(bp.m_tags[pcIndex] != pcTag){
        if(!bp.m_isGlobalHist){
            bp.m_history[pcIndex] = 0 ;
        }

        if(!bp.m_isGlobalTable){
            bp.m_localTable[pcIndex] = vector<state>(power(bp.m_historySize),state(bp.m_state));
        }
        bp.m_tags[pcIndex] = pcTag ;

    }

    if(bp.m_isGlobalHist && !bp.m_isGlobalTable )
    {
        if(bp.m_localTable[pcIndex][bp.m_history[0]]!= STRONGLY_NOT_TAKEN && !taken )
        {
            int ToDec =(int) bp.m_localTable[pcIndex][bp.m_history[0]] ;
            ToDec -- ;
            bp.m_localTable[pcIndex][bp.m_history[0]] = (state)ToDec ;
        }
        if(bp.m_localTable[pcIndex][bp.m_history[0]]!= STRONGLY_TAKEN && taken )
        {
            int ToAdd =(int) bp.m_localTable[pcIndex][bp.m_history[0]] ;
            ToAdd ++ ;
            bp.m_localTable[pcIndex][bp.m_history[0]] = (state)ToAdd ;
        }

        bp.m_history[0] = (bp.m_history[0]*2) % power(bp.m_historySize  )  + taken ;

    }

    if(!bp.m_isGlobalHist && !bp.m_isGlobalTable)
        {
        if (bp.m_localTable[pcIndex][bp.m_history[pcIndex]] != STRONGLY_TAKEN && taken) {
            int ToAdd = (int) bp.m_localTable[pcIndex][bp.m_history[pcIndex]];
            ToAdd++;
            bp.m_localTable[pcIndex][bp.m_history[pcIndex]] = (state) ToAdd;
        }
        if (bp.m_localTable[pcIndex][bp.m_history[pcIndex]] != STRONGLY_NOT_TAKEN && !taken) {
            int ToDec = (int) bp.m_localTable[pcIndex][bp.m_history[pcIndex]];
            ToDec--;
            bp.m_localTable[pcIndex][bp.m_history[pcIndex]] = (state) ToDec;
        }

        bp.m_history[pcIndex] =(bp.m_history[pcIndex] * 2) % power(bp.m_historySize ) + taken;
        }
        uint32_t usedToXor= bitWiseXOR(pc);
    /// GlobalTable
    if(bp.m_isGlobalHist && bp.m_isGlobalTable)
    {
      if(bp.m_globalTable[bp.m_history[0]^ usedToXor] != STRONGLY_NOT_TAKEN && !taken)
      {
          int ToDec =(int) bp.m_globalTable[bp.m_history[0] ^ usedToXor ] ;
          ToDec -- ;
          bp.m_globalTable[bp.m_history[0]^ usedToXor ] = (state)ToDec ;
      }
        if(bp.m_globalTable[bp.m_history[0]^ usedToXor] != STRONGLY_TAKEN && taken)
        {
            int ToAdd =(int) bp.m_globalTable[bp.m_history[0] ^ usedToXor] ;
            ToAdd ++ ;
            bp.m_globalTable[bp.m_history[0] ^ usedToXor ] = (state)ToAdd ;
        }

        bp.m_history[0] = ((bp.m_history[0]*2) % power(bp.m_historySize ) ) + taken ;

    }

    if(!bp.m_isGlobalHist && bp.m_isGlobalTable)
    {
      if(bp.m_globalTable[bp.m_history[pcIndex] ^ usedToXor]!= STRONGLY_TAKEN && taken )
      {
          int ToAdd =(int) bp.m_globalTable[bp.m_history[pcIndex] ^ usedToXor] ;
          ToAdd ++ ;
          bp.m_globalTable[bp.m_history[pcIndex] ^ usedToXor] = (state)ToAdd ;
      }

      if(bp.m_globalTable[bp.m_history[pcIndex] ^ usedToXor]!= STRONGLY_NOT_TAKEN && !taken )
        {
            int ToDec =(int) bp.m_globalTable[bp.m_history[pcIndex] ^ usedToXor] ;
            ToDec -- ;
            bp.m_globalTable[bp.m_history[pcIndex] ^ usedToXor] = (state)ToDec ;
        }

        bp.m_history[pcIndex] =(bp.m_history[pcIndex] * 2) % power(bp.m_historySize ) + taken;

    }

}


// calculate the btb size 
void getBTBSize()
{
   if(bp.m_isGlobalTable && bp.m_isGlobalHist)
    {
        bp.m_simStats.size=bp.m_btbSize * (bp.m_tagSize + 1 + 30);
        bp.m_simStats.size+= 2*power(bp.m_historySize) + bp.m_historySize;
        return;
    }
    else if(bp.m_isGlobalHist && !bp.m_isGlobalTable)
    {
        bp.m_simStats.size=bp.m_historySize;
        bp.m_simStats.size+= bp.m_btbSize* (bp.m_tagSize + 1 + 30 + 2*power(bp.m_historySize));
        return;
    }
    else if(!bp.m_isGlobalHist  && bp.m_isGlobalTable)
    {
        bp.m_simStats.size=2 * power(bp.m_historySize);
        bp.m_simStats.size+= bp.m_btbSize * (bp.m_tagSize + bp.m_historySize + 1 + 30);
        return;
    }
    else {
        bp.m_simStats.size = (2 * power(bp.m_historySize)) + 1 + 30 + bp.m_tagSize +
                                  bp.m_historySize;
        bp.m_simStats.size = bp.m_btbSize * bp.m_simStats.size;
    }

}


void BP_GetStats(SIM_stats *curStats){
    getBTBSize();
    curStats->size = bp.m_simStats.size;
    curStats->flush_num= bp.m_simStats.flush_num;
    curStats->br_num=bp.m_simStats.br_num;
    return;
}


