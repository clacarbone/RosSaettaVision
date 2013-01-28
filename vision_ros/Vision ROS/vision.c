// Steps

#include "globals.h"
#include "couples.h"


void init_all() {


    globals_init();
    
    InitCamera();
    vision_map = vision_map_sphere;
    
    InitPixelMap(vision_map);
    CreateColorStruct(&colorStruct); //struttura contenente le informazione dei colori
    
}



void termination_handler(int signum) {
    
    CloseCamera();
    
    exit(0);
}


void setup_termination() {
    if (signal(SIGINT, termination_handler) == SIG_IGN) signal(SIGINT, SIG_IGN);
    if (signal(SIGHUP, termination_handler) == SIG_IGN) signal(SIGHUP, SIG_IGN);
    if (signal(SIGTERM, termination_handler) == SIG_IGN) signal(SIGTERM, SIG_IGN);
}


void main_loop(){
    int i,j;

    
    matrix_p coord;
    new_matrix(&coord);
    init_matrix(coord,3,1);
    
    
    
    
    // Robot
    
    //
    while (1) {
        // Camera Acquisition
        camera_processing();
        
        
        for (j=0;j<cplList->numCpl;j++) {
            if (coppia_valida(cplList->cpl[j])) {
                coord=cplList->cpl[j].coord;
                printf("Detected Marker: %d\n",j);
                printf("Coordinates: %2.2f %2.2f %2.2f\n ", coord[0][0], coord[1][0], coord[2][0]);
            }
        }

}

void main(int argc, char argv[]) {

    // Setup termination handler
    setup_termination();
    
    // Init All
    init_all();
    
    // Main Loop
    main_loop();
    
    
}