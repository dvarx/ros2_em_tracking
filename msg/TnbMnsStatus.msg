float32[6]       currents_reg        #list of currents flowing in the coils [A]
float32[6]       dclink_voltages     #list of dc link voltages [V]
float32[6]       duties              #list of duty cycles for each channel (buck-stage) [0,1]
float32[6]       res_freqs           #resonant frequencies [Hz]
uint8[6]         states              #states of all channels