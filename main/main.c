/// Steps
#include <stdio.h>
#include <globals.h>
#include <couples.h>
#include <dirent.h>

char		*vision_map;
char		*vision_map_sphere = "data/vision/Mappa90.txt\0";
char		*vision_mode;
char		*vision_mode_sphere = "sphere_\0";

int file_exists(const char * filename)
{
    FILE * file;
    if (file = fopen(filename, "r"))
    {
        fclose(file);
        return 1;
    }
    return 0;
}

void init_all() {


    globals_init();

//    GetLogFileName(log_fname);    
    InitCamera();
//    InitLogs(vision_mode);
    vision_map = vision_map_sphere;
    vision_mode = vision_mode_sphere;
 
    InitPixelMap(vision_map);
    CreateColorStruct(&colorStruct); //struttura contenente le informazione dei colori
    
}



void termination_handler(int signum) {
    
    CloseCamera();
//    CloseLogs();    

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
                printf("Coordinates: %2.2f %2.2f %2.2f\n ", coord->val[0][0], coord->val[1][0], coord->val[2][0]);
            }
        }

    }
}


int main(int argc, char *argv[]) {

    printf ("The current working directory is %s\n", argv[0]);
    // Setup termination handler
    setup_termination();
    chdir("./dist/Debug/GNU-Linux-x86");
    int res = file_exists(INIT_FILE);
    res ? printf("%s exists\n",INIT_FILE) : printf("File doesn't exist\n");
    
    
    /*DIR *dir;
    struct dirent *ent;
    if ((dir = opendir ("./")) != NULL) {
      // print all the files and directories within directory 
      while ((ent = readdir (dir)) != NULL) {
        printf ("%s\n", ent->d_name);
      }
      closedir (dir);
    } else {
      // could not open directory 
      perror ("");
      return EXIT_FAILURE;
    }
    return 0;*/
    // Init All
    init_all();
    
    // Main Loop
    main_loop();
    
}
