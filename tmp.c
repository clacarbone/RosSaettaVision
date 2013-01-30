//
//  vision.c
//
//
//  Created by Andrea Gasparri on 1/28/13.
//
//

//----------------------------------------------------------------------
//ACQUISIZIONE CONDIZIONI INIZIALI
//per le condizioni iniziali assegno le misure in sequenza dalla lista
//----------------------------------------------------------------------
while (!init_done) {
    init_done=1;
    camera_processing();
    if (cplList->numCpl<numRobots)
        init_done=0;
        else {
            for (j=0;j<cplList->numCpl;j++) {
                if (coppia_valida(cplList->cpl[j])) {
                    coord=cplList->cpl[j].coord;
                    //imposto stato iniziale robot
                    set_initial_state(&(robList->robots[j]),coord);
                    //imposto misure robot
                    update_measure(&(robList->robots[j]),coord);
                }
                else {
                    init_done=0;
                }
            }
        }
}


//----------------------------------------------------------------------
//ASSEGNAZIONE IP AI ROBOT
//----------------------------------------------------------------------



//----------------------------------------------------------------------
// ACQUISIZIONE POSIZIONE ROBOT
//----------------------------------------------------------------------


if (!first_iter) {	//alla prima iterazione salto acquisizione perché già fatta per cond iniziali
    gettimeofday(&bef_proc,NULL);
    camera_processing();
    gettimeofday(&aft_proc,NULL);
	
	Tc_Camera=(aft_proc.tv_sec-bef_proc.tv_sec)+(aft_proc.tv_usec-bef_proc.tv_usec)/1000000.0;
    for (j=0;j<robList->numRob;j++) {
        cnst(HUGE_VALF,robList->robots[j].measure);
        float v_temp=robList->robots[j].input_prev->val[0][0];
        float w_temp=robList->robots[j].input_prev->val[1][0];
        float theta_temp=robList->robots[j].state->val[2][0];
        robList->robots[j].state->val[0][0]+=v_temp*cos(theta_temp)*Tc_Camera;
        robList->robots[j].state->val[1][0]+=v_temp*sin(theta_temp)*Tc_Camera;
		robList->robots[j].state->val[2][0]+=w_temp*Tc_Camera;
    }
    //associo ogni misura al robot giusto
    for (j=0;j<cplList->numCpl;j++) {
        index_cpl=minima_distanza_from_robot(robList,cplList,j);
        if (index_cpl!= -1) {
            update_measure(&(robList->robots[index_cpl]),cplList->cpl[j].coord);
		    robList->robots[index_cpl].state->val[0][0]=(robList->robots[index_cpl].state->val[0][0]+cplList->cpl[j].coord->val[0][0])/2;
		    robList->robots[index_cpl].state->val[1][0]=(robList->robots[index_cpl].state->val[1][0]+cplList->cpl[j].coord->val[1][0])/2;
		    robList->robots[index_cpl].state->val[2][0]=(robList->robots[index_cpl].state->val[2][0]+cplList->cpl[j].coord->val[2][0])/2;
            //update_state(&(robList->robots[index_cpl]),cplList->cpl[j].coord);
        }
    }
}

