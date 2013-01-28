#include "main.h"
#include "kinectPackages.h"

FILE *data;
FILE *pipe_a;

const int NUM_OBSTACLES=0;
int continue_loop=1;
int msgsockFD;
int sockFD;
kin_vel *velocities;
void *buffer;
pthread_mutex_t mutex_v;
pthread_mutex_t mutex_param;
struct sockaddr_in nuovo_client;

void kinect_communication_init()
{
    struct sockaddr_in server;

    sockFD = socket ( AF_INET, SOCK_STREAM, 0 );

    if ( sockFD < 0 )
    {
        printf ( "errore in socket()\n" );
        exit ( 1 );
    }

    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons ( MYPORT );

    /* lego la socket al numero di porta */
    int opt=TRUE;
    setsockopt ( sockFD,SOL_SOCKET,SO_REUSEADDR, ( char* ) &opt,sizeof ( opt ) );
    if ( bind ( sockFD, ( struct sockaddr* ) &server, sizeof ( server ) ) )
    {
        printf ( "errore in bind(): %s\n",strerror ( errno ) );
        exit ( 1 );
    }

    /* dimensionamento coda di backlog */

    if ( listen ( sockFD,5 ) < 0 )
    {
        printf ( "errore in listen()\n" );
        exit ( 1 );
    }

    //int flags;
    //flags = fcntl(sockFD,F_GETFL,0);
    //fcntl(sockFD, F_SETFL, flags | O_NONBLOCK);

    velocities=malloc ( sizeof ( kin_vel ) );
    pthread_mutex_init ( &mutex_v, NULL );
    pthread_mutex_init ( &mutex_param, NULL);
    pthread_mutex_lock(&mutex_v);
    velocities->v_x=0.0;
    velocities->v_y=0.0;
    pthread_mutex_unlock(&mutex_v);
}

void close_communication()
{
    pthread_mutex_destroy ( &mutex_v );
    pthread_mutex_destroy ( &mutex_param );
    if (velocities)
        free ( velocities );
    if (buffer)
        free ( buffer );
    if (sockFD)
        close(sockFD);
}

void* tf_kinect ( void *arguments )
{
    int recvMsgSize;                    /* Size of received message */
    uint8 *sizes= ( uint8* ) malloc ( sizeof ( uint8 ) *2 );
    /* Receive message from client */
    unsigned int size;
    size = sizeof ( struct sockaddr_in );

    while ( continue_loop )
    {
        msgsockFD = accept ( sockFD, ( struct sockaddr* ) &nuovo_client, &size );
        printf ( "beep\n" );
        if ( msgsockFD<0 )
        {
            printf ( "errore in accept() or no connection  from kinect: %s\n",strerror ( errno ) );
            exit ( 1 );
        }
        recvMsgSize = recv ( msgsockFD, ( void* ) sizes, sizeof ( uint8 ) *2, 0 );
        if ( recvMsgSize < 0 )
        {
            printf ( "error in recv(): %s\n",strerror ( errno ) );

        }
        else
        {
            buffer=malloc ( sizes[1] );
            recvMsgSize = recv ( msgsockFD, buffer, sizes[1], 0 );
            if ( recvMsgSize < 0 )
            {
                printf ( "error in recv(): %s\n",strerror ( errno ) );
            }
            else
            {
                switch ( sizes[0] )
                {
                case KIN_SPEEDS:
                    pthread_mutex_lock ( &mutex_v );
                    velocities->v_x= ( ( kin_vel* ) buffer )->v_x;
                    velocities->v_y= ( ( kin_vel* ) buffer )->v_y;
                    pthread_mutex_unlock ( &mutex_v );
                    printf ( "new vels %f %f\n",velocities->v_x,velocities->v_y );
                    break;
                case KIN_PARAMS:
                    pthread_mutex_lock(&mutex_param);
                    a*= ( ( kin_param* ) buffer )->a;
                    b*= ( ( kin_param* ) buffer )->b;
                    c*= ( ( kin_param* ) buffer )->c;
                    pthread_mutex_unlock(&mutex_param);
                    printf ( "new params %f %f %f\n",a,b,c );
                    break;
                }
            }
            free(buffer);
        }
        close ( msgsockFD );

    }
    free ( sizes );
}

/* Calcola angolo di (x1,y1) rispetto a (x2,y2)*/
float angle_between_points(float x1,float y1,float x2,float y2) {
    return atan2f(y2-y1,x2-x1);
}
//======================================================================

/* Inizializza lista robot */
void RobotSetup(robotList **robList) {
    int i;
    *robList = (robotList *)malloc( sizeof( robotList ) );
    (*robList)->numRob=numRobots;
    (*robList)->robots = (robot_p)malloc(sizeof(robot_t)*(*robList)->numRob);


    for (i=0;i<(*robList)->numRob;i++) {
        new_matrix(&((*robList)->robots[i].state));
        new_matrix(&((*robList)->robots[i].state_prev));
        new_matrix(&((*robList)->robots[i].measure));
        new_matrix(&((*robList)->robots[i].input));
        new_matrix(&((*robList)->robots[i].input_prev));
        init_robot(&((*robList)->robots[i]),i);
    }
}
//======================================================================

/* Inizializza lista kalman */
void KalmanSetup(kalmanList **kList, robotList **robList) {
    int i;
    *kList=(kalmanList *)malloc(sizeof(kalmanList));
    (*kList)->numKalman=numRobots;
    (*kList)->kalmans = (kalman_p)malloc(sizeof(kalman_t)*(*kList)->numKalman);
    for (i=0;i<(*kList)->numKalman;i++) {
        new_matrix(&((*kList)->kalmans[i].est_state));
        new_matrix(&((*kList)->kalmans[i].est_state_prev));
        new_matrix(&((*kList)->kalmans[i].est_state_minus));
        new_matrix(&((*kList)->kalmans[i].evol));
        new_matrix(&((*kList)->kalmans[i].A));
        new_matrix(&((*kList)->kalmans[i].W));
        new_matrix(&((*kList)->kalmans[i].H));
        new_matrix(&((*kList)->kalmans[i].V));
        new_matrix(&((*kList)->kalmans[i].Q_prev));
        new_matrix(&((*kList)->kalmans[i].R));
        new_matrix(&((*kList)->kalmans[i].P));
        new_matrix(&((*kList)->kalmans[i].P_prev));
        new_matrix(&((*kList)->kalmans[i].P_minus));
        new_matrix(&((*kList)->kalmans[i].K));
        init_kalman(&((*kList)->kalmans[i]),&((*robList)->robots[i]),SAMPLE_TIME);
        kalman_initialization(&((*kList)->kalmans[i]));
    }
}
//======================================================================

/* Inizializza lista controllori */
void FormControllerSetup(controllerList **ctrlList, robotList **robList) {
#ifdef FORM_CTRL
    int i;

    //controllore --> uno per ogni robot
    *ctrlList=(controllerList *)malloc(sizeof(controllerList));
    (*ctrlList)->numController=numRobots;
    (*ctrlList)->controllers = (controller_p)malloc(sizeof(controller_t)*(*ctrlList)->numController);


    //lettura parametri da file (tipo form e dist)
    FILE *ffi;
    ffi=fopen(CTRL_INIT_FILE,"r");
    if (ffi==NULL) {
        printf("Error on file opening\n");
        exit(-1);
    }
    fscanf(ffi,"%d\n%f\n",&f_type,&d_rob);
    fclose(ffi);

    //per ogni controllore
    for (i=0;i<(*ctrlList)->numController;i++) {
        (*ctrlList)->controllers[i].d=d_rob;
        //calcolo punto controllato robot
        calcola_pt_ctrl(&((*ctrlList)->controllers[i]),&((*robList)->robots[i]));
        //all'inizio robot fermo
        (*ctrlList)->controllers[i].v=0;
        (*ctrlList)->controllers[i].w=0;
    }
    //--------------------------------------------------------------------------------------------------

    //lettura da file dati formazione
    char str[50];	//file formazione
    char pt_ctr;
    //char ipP[15];	//togliere: non usato

    char ip1[15];

    FILE *ff;
    //------------------------------------------------------------------
    //------------------------------------------------------------------
    //IMPOSTAZIONE PARAMETRI FORMAZIONE
    //0 -->	linea
    //1 --> triangolo
    //2 --> diagonale
    //3 --> singolo
    //------------------------------------------------------------------
    //_-----------------------------------------------------------------
    //MODIFICARE STRCMP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


    switch (f_type) {
        //------LINEA---------------------------------------------------------
    case 0: {//linea
        char ip2[15];
        float dist;
        sprintf(str,"%slinea.txt",FORM_DIR);
        ff=fopen(str,"r");
        if (ff==NULL) {
            printf("Error on file opening\n");
            exit(-1);
        }
        fscanf(ff,"%s\n%s\n%f\n",ip1,ip2,&dist);
        fclose(ff);

        /*
        //calcolo baricentro formazione
        (*form)->x_b=0;
        (*form)->y_b=0;
        for(i=0;i<(*robList)->numRob;i++){
        	(*form)->x_b=(i*(*form)->x_b+(*robList)->robots[i].state->val[0][0])/i+1;
        	(*form)->y_b=(i*(*form)->y_b+(*robList)->robots[i].state->val[1][0])/i+1;
        }

        //calcolo angolo formazione
        float x_temp,y_temp; //per calcolo angolo formazione
        for(i=0;i<(*robList)->numRob;i++){
        	if((*robList)->robots[i].ip==ipP){
        		x_temp=(*robList)->robots[i].state->val[0][0];
        		y_temp=(*robList)->robots[i].state->val[1][0];
        	}
        }
        //è l'angolo tra il baricentro e il pilota
        (*form)->fi_c=atan2f(y_temp-(*form)->y_b,x_temp-(*form)->x_b);

        //calcolo punto controllato formazione
        (*form)->x_c=(*form)->x_b+(*form)->d*cosf((*form)->fi_c);
        (*form)->y_c=(*form)->y_b+(*form)->d*sinf((*form)->fi_c);
        //all'inizio il pt desiderato coincide con il pt ctrl attuale
        (*form)->x_d=(*form)->x_c;
        (*form)->y_d=(*form)->y_c;
        //(*form)->fi_d=(*form)->fi_c;
        //*/
        //per ogni controllore
        for (i=0;i<(*ctrlList)->numController;i++) {
            //PRIMO
            if (strcmp((*robList)->robots[i].ip,ip1)==0) {
                //dist pt ctrl
                (*ctrlList)->controllers[i].l=0;
                //angolo
                (*ctrlList)->controllers[i].angle=0;
            }
            //SECONDO
            if (strcmp((*robList)->robots[i].ip,ip2)==0) {
                //dist pt ctrl
                (*ctrlList)->controllers[i].l=dist;
                //angolo
                (*ctrlList)->controllers[i].angle=3.0*M_PI/2.0;
            }
        }
    }
    //------TRIANGOLO--------------------------------------------------------------
    case 1:	{//triangolo
        float lato, angle_vert;
        char ipA[15],ipB[15];	//togliere: non usato
        char ip2[15], ip3[15];

        sprintf(str,"%striangolo.txt",FORM_DIR);
        ff=fopen(str,"r");
        if (ff==NULL) {
            printf("Error on file opening\n");
            exit(-1);
        }
        fscanf(ff,"%s\n%s\n%s\n%f\t%f\n",ip1,ip2,ip3,&lato,&angle_vert);
        fclose(ff);
        //angolo al vertice del triangolo
        angle_vert=DGR_TO_RAD(angle_vert);

        //per ogni controllore
        for (i=0;i<(*ctrlList)->numController;i++) {
            //ROBOT DAVANTI
            if ((*robList)->robots[i].ip==ip1) {
                //dist pt ctrl
                (*ctrlList)->controllers[i].l=0;
                //angolo
                (*ctrlList)->controllers[i].angle= 0;
            }

            //ROBOT A (SX)
            if ((*robList)->robots[i].ip==ip2) {
                //dist pt ctrl
                (*ctrlList)->controllers[i].l=lato;
                //angolo
                (*ctrlList)->controllers[i].angle=(3.0/2.0)*M_PI - angle_vert;
            }

            //ROBOT B (DX)
            if ((*robList)->robots[i].ip==ip3) {
                //dist pt ctrl
                (*ctrlList)->controllers[i].l=lato;
                //angolo
                (*ctrlList)->controllers[i].angle=(3.0/2.0)*M_PI + angle_vert;
            }
        }
    }
    //------DIAGONALE--------------------------------------------------------------
    case 2:	{//diagonale
        float lato, angle_vert;
        char ipA[15],ipB[15];	//togliere: non usato
        char ip2[15], ip3[15];

        sprintf(str,"%sdiagonale.txt",FORM_DIR);
        ff=fopen(str,"r");
        if (ff==NULL) {
            printf("Error on file opening\n");
            exit(-1);
        }
        fscanf(ff,"%s\n%s\n%s\n%f\t%f\n",ip2,ip2,ip3,&lato,&angle_vert);
        fclose(ff);
        //angolo al vertice del triangolo
        angle_vert=DGR_TO_RAD(angle_vert);

        //per ogni controllore
        for (i=0;i<(*ctrlList)->numController;i++) {
            //ROBOT DAVANTI
            if ((*robList)->robots[i].ip==ip1) {
                //dist pt ctrl
                (*ctrlList)->controllers[i].l=0;
                //angolo
                (*ctrlList)->controllers[i].angle= 0;
            }

            //ROBOT DIETRO
            if ((*robList)->robots[i].ip==ip2) {
                //dist pt ctrl
                (*ctrlList)->controllers[i].l=lato;
                //angolo
                (*ctrlList)->controllers[i].angle=(3.0/2.0)*M_PI - angle_vert;
            }
        }

        case 3: {//singolo
            sprintf(str,"%ssingolo.txt",FORM_DIR);
            ff=fopen(str,"r");
            if (ff==NULL) {
                printf("Error on file opening\n");
                exit(-1);
            }
            fscanf(ff,"%s\n",ip1);
            fclose(ff);

            (*ctrlList)->controllers[0].l=0;
            //angolo
            (*ctrlList)->controllers[0].angle=0;
        }
        //----------------------------------------------------------------------
        break;
    }
    default:
        printf("Errore: formazione non riconosciuta\n");
        exit(-1);
    }
#endif
}
//======================================================================

/* Inizializza lista controllori */
void SwarmControllerSetup(controllerList **ctrlList, robotList **robList) {
    int i;

    //controllore --> uno per ogni robot
    *ctrlList=(controllerList *)malloc(sizeof(controllerList));
    (*ctrlList)->numController=numRobots;
    (*ctrlList)->controllers = (controller_p)malloc(sizeof(controller_t)*(*ctrlList)->numController);


    //lettura parametri da file (tipo form e dist)
    FILE *ffi;
    ffi=fopen(SWARM_PARAM_FILE,"r");
    if (ffi==NULL) {
        printf("Error on file opening\n");
        exit(-1);
    }
    fscanf(ffi,"%f\n",&d_rob);//,&a,&b,&c);
    fclose(ffi);

    //per ogni controllore
    for (i=0;i<(*ctrlList)->numController;i++) {
        (*ctrlList)->controllers[i].d=d_rob;
        //calcolo punto controllato robot
        calcola_pt_ctrl(&((*ctrlList)->controllers[i]),&((*robList)->robots[i]));
        //all'inizio robot fermo
        (*ctrlList)->controllers[i].v=0;
        (*ctrlList)->controllers[i].w=0;
    }
    //--------------------------------------------------------------------------------------------------

    //Passino parameters
    pthread_mutex_lock(&mutex_param);
    //a=.03;
    //b=.10;
    //c=.3;
    a=.4;
    b=.2;
    c=.2;
    pthread_mutex_unlock(&mutex_param);
    a_obst=.03;
    b_obst=.3;
    c_obst=.4;
    ////////////////////
}


void main_init() {

    int i;
#ifdef WEBCAM
    globals_init();
#ifdef PRINT
    printf("VISION FLAG:\nSAVE_FRAMES\t %d\nSAVE_NORMALIZED\t %d\nSAVE_BINARY\t %d\nSAVE_COLOR_IMG\t %d\nSAVE_CENTROIDS\t %d\nn_save_frame\t %d\nDEFAULT_HOR_RESOLUTION\t %d\nDEFAULT_VERT_RESOLUTION\t %d\nDEFAULT_FPS\t %d\nLOGGING\t %d\n", SAVE_FRAMES, SAVE_NORMALIZED, SAVE_BINARY, SAVE_COLOR_IMG, SAVE_CENTROIDS, n_save_frame, DEFAULT_HOR_RESOLUTION, DEFAULT_VERT_RESOLUTION, DEFAULT_FPS, LOGGING);
    printf("BLOB SIZE: MIN %d\tMAX %d\n",BLOB_MIN_SIZE,BLOB_MAX_SIZE);
#endif
    GetLogFileName(log_fname);
    InitCamera();
    vision_map = vision_map_sphere;
    vision_mode = vision_mode_sphere;

    InitPixelMap(vision_map);
    InitLogs(vision_mode);
    CreateColorStruct(&colorStruct); //struttura contenente le informazione dei colori

#endif
    //init_ekf();
    //Initialization of the obstacles
    kinect_communication_init();
    obstacles=(float**)malloc(sizeof(float*)*NUM_OBSTACLES);
    for (i=0;i<NUM_OBSTACLES;i++)
    {
        //2 is the dimension of the space the obstacle belongs to.
        obstacles[i]=(float*)malloc(sizeof(float)*2);
    }

    //By now, the coordinates are hardcoded. Next_time they will be loaded by a configuration file...
    /*obstacles[0][0]=60;
    obstacles[0][1]=120;
    obstacles[1][0]=120;
    obstacles[1][1]=180;
	*/
}
//======================================================================

void termination_handler(int signum) {

    int i;
    continue_loop=0;

#ifdef WIFI
//fermo tutti i robot
    int j;
    for (j=0;j<robList->numRob;j++) {
        send_wifi_msg(sock[j],MSG_STOP);
    }
    free(sock);
#endif

    for (i=0;i<NUM_OBSTACLES;i++)
    {
        //2 is the dimension of the space the obstacle belongs to.
        free(obstacles[i]);
    }
    free(obstacles);
//close_ekf();

    close_communication();
    fclose(pipe_a);
    fclose(data);

#ifdef WEBCAM
    CloseCamera();
    CloseLogs();
#endif



    exit(0);
}
//======================================================================

void setup_termination() {
    if (signal(SIGINT, termination_handler) == SIG_IGN) signal(SIGINT, SIG_IGN);
    if (signal(SIGHUP, termination_handler) == SIG_IGN) signal(SIGHUP, SIG_IGN);
    if (signal(SIGTERM, termination_handler) == SIG_IGN) signal(SIGTERM, SIG_IGN);
}
//======================================================================

void* tf_main(void* thread_arg) {
    setup_termination();
    int i;
    float Tc_Camera=0.0;
//    float u_kalman[2];


    FILE *ff, *fm, *fs;
    pipe_a = popen("gnuplot -noraise","w");

    fprintf(pipe_a, "set samples 100000\n");
    //fprintf(pipe_a, "set terminal wxt size 600,600\n");
    fprintf(pipe_a, "set hidden3d\n");
    fprintf(pipe_a, "set xrange [-400:400]\n");
    fprintf(pipe_a, "set yrange [-400:400]\n");
    fprintf(pipe_a, "set term x11\n");
    //fprintf(pipe_a, "set output 'graph.png'\n");
    fprintf(pipe_a, "set title 'We are plotting from C'\n");
    fprintf(pipe_a, "set xlabel 'Label X'\n");
    fprintf(pipe_a, "set ylabel 'Label Y'\n");
    /* fprintf(pipe_a, "plot 'datafile.dat' using 1:2\n");*/
//variabili per calcolo tempo ciclo
    struct timeval t0,t1;
    struct timeval bef_proc,aft_proc;
    long int time0,time1;
    gettimeofday(&t1,NULL);
    double period;
    double p0,p1,p2,pt;	//period vision, kalman, controllore, totale

    struct timeval tm0, tm1, tm2, tm3;
//struct timeval t0_v,t1_v,t0_k,t1_k,t0_c,t1_c;
//gettimeofday(&t1_v,NULL);
//gettimeofday(&t1_k,NULL);
//gettimeofday(&t1_c,NULL);

#ifdef WEBCAM
    int init_done=0;
    int first_iter=1;
    int index_cpl;

    RobotSetup(&robList);
    contatore22=0;

    int j;
    matrix_p coord;
    new_matrix(&coord);
    init_matrix(coord,3,1);

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

//Inizializzazione EKF
    /*for(j=0;j<robList->numRob;j++){
    	x_pred[j].x=cplList->cpl[j].coord->val[0][0];
    	x_pred[j].y=cplList->cpl[j].coord->val[1][0];
    	x_pred[j].th=cplList->cpl[j].coord->val[2][0];
    	x_post[j].x=x_pred[j].x;
    	x_post[j].y=x_pred[j].y;
    	x_post[j].th=x_pred[j].th;
    }
    */


//----------------------------------------------------------------------

//----------------------------------------------------------------------
//ASSEGNAZIONE IP AI ROBOT
//----------------------------------------------------------------------
    char stessa_conf;
    int scelta_valida=0;

    printf("\n\n\n\nASSEGNAZIONE IP AI ROBOT:\n");
    printf("Utilizzare configurazione esistente? [s/n]\n");

    while (!scelta_valida) {
        scanf("%c",&stessa_conf);
        if (stessa_conf=='s' || stessa_conf=='n') {
            scelta_valida=1;
        }
        else	printf("Scelta non valida, ripetere\n");
    }
    FILE *rob_ip_file;
    if (stessa_conf=='s') {
        rob_ip_file=fopen(ROBOT_IP_FILE,"r");
        if (rob_ip_file==NULL) {
            printf("Error on file opening\n");
            printf("Impossibile aprire il file di configurazione, procedere con assegnazione ip\n");
            stessa_conf='n';
        }
        for (j=0;j<robList->numRob;j++)
            fscanf(rob_ip_file,"%s\n",robList->robots[j].ip);
        //fclose(rob_ip_file);
    }

    if (stessa_conf=='n') {
        FILE *ipff;
        ipff=fopen(IP_FILE,"r");
        if (ipff==NULL)
            printf("Error on file opening\n");

        rob_ip_file=fopen(ROBOT_IP_FILE,"w");
        if (rob_ip_file==NULL)
            printf("Error on file opening\n");

        int num_ip=0;
        int ch;
        while ((ch=fgetc(ipff))!=EOF)
            if (ch=='\n')
                num_ip++;
        fclose(ipff);

        char *ipList[num_ip];

        ipff=fopen(IP_FILE,"r");
        if (ipff==NULL)
            printf("Error on file opening\n");
        for (j=0;j<num_ip;j++) {
            ipList[j]=malloc(15*sizeof(char));
            fscanf(ipff,"%s\n",ipList[j]);
        }
        fclose(ipff);

        int k=0;
        int scelta_ip=0;

        int bl[num_ip];
        for (j=0;j<num_ip;j++)
            bl[j]=0;
        for (j=0;j<robList->numRob;j++) {
            printf("\n\nIl robot %d si trova in posizione [%f,%f],\nscegliere indirizzo ip:\n", j, robList->robots[j].state->val[0][0],robList->robots[j].state->val[1][0]);
            for (k=0;k<num_ip;k++) {
                if (bl[k]==0)
                    printf("\t%d)\t %s \n",k,ipList[k]);
            }
            scanf ("%d",&scelta_ip);
            while (bl[scelta_ip]==1 || scelta_ip>=num_ip) {
                printf("Scelta non valida. Ripetere\n");
                scanf("%d",&scelta_ip);
            }
            bl[scelta_ip]=1;
            sprintf(robList->robots[j].ip,ipList[scelta_ip]);
            fprintf(rob_ip_file,"%s\n",ipList[scelta_ip]);
        }
    }
    fclose(rob_ip_file);

    for (j=0;j<robList->numRob;j++) {
        printf("ROBOT %d\t%s\n",j,robList->robots[j].ip);
    }
    //*/
#endif
//----------------------------------------------------------------------
//FINE PROC
//----------------------------------------------------------------------


//----------------------------------------------------------------------
//INIZIALIZZO CONNESSIONE CON ROBOT
//----------------------------------------------------------------------
#ifdef CON_ROBOT
    sock=(int*)malloc(sizeof(int)*(robList->numRob));
    for (j=0;j<robList->numRob;j++)
        sock[j]=init_wifi(robList->robots[j].ip);
#endif
//----------------------------------------------------------------------

//----------------------------------------------------------------------

    /* // TEST INVIO VEL
    printf("INVIO VEL\n");
    getchar();
    int sock[robList->numRob];
    v=10;
    w=0;
    for(j=0;j<robList->numRob;j++)
    	sock[j]=init_wifi(robList->robots[j].ip);
    for(j=0;j<robList->numRob;j++){
    send_wifi_vel(sock[j],v,w);
    getchar();
    send_wifi_msg(sock[j],MSG_STOP);
    printf("\nMSG STOP %d\n",MSG_STOP);
    getchar();
    }
    //*/

    KalmanSetup(&kList,&robList);

#ifdef FORM_CTRL
    FormControllerSetup(&ctrlList,&robList);
#endif
#ifdef SWARM_CTRL
    SwarmControllerSetup(&ctrlList,&robList);
#endif

    /*
    for(i=0;i<robList->numRob;i++){
    printf("ROB IP %d	%s\n",i,robList->robots[i].ip);

    printf("TEST CONTROLLORE	%d\n",i);
    printf("\tV %f\t W %f\n\tD %f\n",ctrlList->controllers[i].v,ctrlList->controllers[i].w,ctrlList->controllers[i].l);
    }
    */

//======================================================================
    int p;
//esecuzione
    /*
    #ifdef WIFI
    //preparo comunicazione
    int sock[robList->numRob];
    for(j=0;j<robList->numRob;j++)
    	sock[j]=init_wifi(robList->robots[i].ip);
    #endif
    //*/

#ifdef PRINT
    printf("INIT_DONE\n");
#endif
//getchar();
#ifdef FORM_CTRL
    viapoint_file=fopen(VIAPOINT_FILE,"r");
    leggi_posizione_desiderata();
#endif

#ifdef LOG
    remove(LOG_FILE1);
    remove(LOG_FILE2);
#endif


    int conta=0;
    while (1) {
        gettimeofday(&t0,NULL);
        //PRIMO TEMPO
        gettimeofday(&tm0,NULL);
        if (t0.tv_sec==t1.tv_sec)
            period=(t0.tv_usec-t1.tv_usec)/(double)1000000;
        else
            period=(1000000-(t0.tv_usec-t1.tv_usec))/(double)1000000;
        t1=t0;

        for (p=0;p<kList->numKalman;p++) {
            kList->kalmans[p].sample_time=period;
        }

#ifdef WEBCAM

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
                /*for(j=0;j<robList->numRob;j++){
                	//ad ogni iterazione annullo le misure --> tutte inf
                	cnst(HUGE_VALF,robList->robots[j].measure);
                	//per ogni robot confronto lo stato stimato con le nuove misure e scelgo quella che si discosta meno
                	//printf("TEST PT %f, %f\n",robList->robots[j].state->val[0][0],robList->robots[j].state->val[1][0]);	//già -nan

                	//printf("ROBOT\n");
                	//print_matrix(robList->robots[j].state);
                	//printf("\nKALMAN\n");
                	//print_matrix(kList->kalmans[j].est_state);
                	//getchar();


                	index_cpl=minima_distanza(robList->robots[j].state,cplList);

                //	printf("index couple %d",index_cpl);
                	//aggiorno le misure
                	if(index_cpl!= -1){
                	update_measure(&(robList->robots[j]),cplList->cpl[index_cpl].coord);

                	//SKIPPO KALMAN!!!
                	update_state(&(robList->robots[j]),cplList->cpl[index_cpl].coord);
                	}
                */
                //printf("\n");

                //printf("ROB %d, %f,\t%f\n",j,robList->robots[j].measure->val[0][0],robList->robots[j].measure->val[1][0]);
            }
            //getchar();

        }

        first_iter=0;
        int i;

        //SECONDO TEMPO
        gettimeofday(&tm1,NULL);



        //ATTILIO: Qui ci va kalman!

        /*for(j=0;j<robList->numRob;j++){
        	float meas[3];
        	u_kalman[0]=robList->robots[j].input_prev->val[0][0];
        	u_kalman[1]=robList->robots[j].input_prev->val[1][0];
        	//Prediction
        	EKF_Prediction(x_post[j], &(x_pred[j]), u_kalman);
        	//Correction

        	meas[0]=robList->robots[j].measure->val[0][0];
        	meas[1]=robList->robots[j].measure->val[1][0];
        	meas[2]=robList->robots[j].measure->val[2][0];
        	if(isnan(meas[0]) ||	isnan(meas[1])	||	isnan(meas[2]))
        		EKF_Correction(x_pred[j],&(x_post[j]),(float *)meas);
        	else {
        		x_post[j].x=x_pred[j].x;
        		x_post[j].y=x_pred[j].y;
        		x_post[j].th=x_pred[j].th;
        	}

        	printf("\nMEAS: %f %f %f \n",meas[0],meas[1],meas[2]);
        	printf("XPRE: %f %f %f \n",x_pred[j].x,x_pred[j].y,x_pred[j].th);
        	printf("XPOS: %f %f %f \n",x_post[j].x,x_post[j].y,x_post[j].th);
        }*/
//	for(i=0;i<kList->numKalman;i++){
        //printf("KALMAN %d\n",i);
        //	print_matrix(kList->kalmans[i].est_state);
        //printf("aaaa\n");
        //	kalman_filter(&(kList->kalmans[i]));
//		printf("KALMAN %d\n",i);
//			print_matrix(kList->kalmans[i].est_state);a

/////////////////////////////////////7NO
        /*DATI KALMAN SU FILE - PER TEST
        fm=fopen(MISURE_KALMAN_FILE,"a");
        if (!fm)
        fprintf(stderr,"Errore file misure!!!\n");
        fs=fopen(STATO_KALMAN_FILE,"a");
        if (!fs)
        	fprintf(stderr,"Errore file stato!!!\n");

        fprint_vector_inrow(fm,kList->kalmans[0].rob->measure);
        fprint_vector_inrow(fs,kList->kalmans[0].est_state);
        fclose(fm);
        fclose(fs);
        //*/
//////////////////////////////////////7
//	}
        //getchar();


        //TERZO TEMPO
        gettimeofday(&tm2,NULL);

#ifdef FORM_CTRL
        //Posizione desiderata per la formazione
        float x_att=robList->robots[0].state->val[0][0];
        float y_att=robList->robots[0].state->val[1][0];
        float dist_att_des=sqrtf(powf(x_att-x_des_form,2.0)+powf(y_att-y_des_form,2.0));
        //se la pos des è stata raggiunta
        if (dist_att_des<SOGLIA_POS_DES) {
            //leggi nuovo punto
            leggi_posizione_desiderata();
        }
        //calcola input
        for (i=0;i<ctrlList->numController;i++) {
            calcola_pt_ctrl(&(ctrlList->controllers[i]),&(robList->robots[i]));
            calcola_input(&(ctrlList->controllers[i]),&(robList->robots[i]));
        }
#endif

#ifdef SWARM_CTRL
        float ux, uy;
        float xi, xj, yi, yj;
        float cosfi, sinfi;
        float normaquadra_y;
        int j;
        float equilibrium=sqrtf(c*logf(b/a));
        float obstacles_error=0;
        int obst_index=0;

        data =fopen("data.dat","w");
        for (i=0;i<robList->numRob;i++) {
            fprintf(data,"%f\t%f\n",robList->robots[i].state->val[0][0],robList->robots[i].state->val[1][0]);
            fprintf(pipe_a,"set arrow %d from %f,%f to %f,%f\n",i+1,robList->robots[i].state->val[0][0],robList->robots[i].state->val[1][0],robList->robots[i].state->val[0][0]+15*cos(robList->robots[i].state->val[2][0]),robList->robots[i].state->val[1][0]+15*sin(robList->robots[i].state->val[2][0]));
            //fprintf(pipe_a, "plot \"< echo '%d %d'\"\n",i,i);
        }
        fclose(data);
        fprintf(pipe_a,"set multiplot\n");
        fprintf(pipe_a,"plot [-400:400][-400:400] \"data.dat\" using 1:2\n");
        fflush(pipe_a);

        FILE *fp;
        for (i=0;i<robList->numRob;i++) {
            obstacles_error=0;
            ux=0;
            uy=0;
            xi=robList->robots[i].state->val[0][0]/100.0;
            yi=robList->robots[i].state->val[1][0]/100.0;
	float	a_1=0.4;
	float	b_1=0.24;	
            for (j=0;j<robList->numRob;j++) {
                if (j!=i) {
                    xj=robList->robots[j].state->val[0][0]/100.0;
                    yj=robList->robots[j].state->val[1][0]/100.0;

                    normaquadra_y=sqrtf((xi-xj)*(xi-xj)+(yi-yj)*(yi-yj))-0.1;//powf((xi-xj),2)+powf((yi-yj),2);
                    if (normaquadra_y<1.5) {
                        pthread_mutex_lock(&mutex_param);
                        ux += -(xi-xj)*(a_1-b_1/normaquadra_y);
                        uy += -(yi-yj)*(a_1-b_1/normaquadra_y);
                        pthread_mutex_unlock(&mutex_param);
                    }
                }
            }

            for (obst_index=0;obst_index<NUM_OBSTACLES;obst_index++)
            {
                float obst_coord_x,obst_coord_y;
                obst_coord_x=obstacles[obst_index][0]/100.0;
                obst_coord_y=obstacles[obst_index][1]/100.0;
                obstacles_error=sqrtf((xi-obst_coord_x)*(xi-obst_coord_x)+(yi-obst_coord_y)*(yi-obst_coord_y));
                if (obstacles_error<=equilibrium-0.03)
                {
                    ux += -(xi-obst_coord_x)*(a_obst-b_obst*exp(-obstacles_error/c_obst));
                    uy += -(yi-obst_coord_y)*(a_obst-b_obst*exp(-obstacles_error/c_obst));
                }
            }

            ux*=100;
            uy*=100;

            pthread_mutex_lock(&mutex_v);
            ux+=velocities->v_x;
            uy+=velocities->v_y;
            pthread_mutex_unlock(&mutex_v);

            if (velocities->v_x==0 && velocities->v_y==0)
            {
                ux=0.0;
                uy=0.0;
            }
            //uy+=2;
            calcola_pt_ctrl(&(ctrlList->controllers[i]),&(robList->robots[i]));
            cosfi=cosf(robList->robots[i].state->val[2][0]);	//coseno angolo robot
            sinfi=sinf(robList->robots[i].state->val[2][0]);	//seno angolo robot
            ctrlList->controllers[i].v=cosfi*ux+sinfi*uy;// + V_DES;
            ctrlList->controllers[i].w=(1/ctrlList->controllers[i].d)*(-sinfi*ux+cosfi*uy);// + W_DES;
            fp=fopen(PASSINO_FILE,"a");
            fprintf(fp,"%f\t%f\n",ctrlList->controllers[i].v,ctrlList->controllers[i].w);
            fclose(fp);

            /*
            if(i==0){
            	ctrlList->controllers[i].v=0.0;
            	ctrlList->controllers[i].w=0.0;
            }*/
            float lim_v=8.0;
            float lim_w=0.5;

            if (ctrlList->controllers[i].v>lim_v)
                ctrlList->controllers[i].v=lim_v;
            if (ctrlList->controllers[i].v<-lim_v)
                ctrlList->controllers[i].v=-lim_v;
            if (ctrlList->controllers[i].w>lim_w)
                ctrlList->controllers[i].w=lim_w;
            if (ctrlList->controllers[i].w<-lim_w)
                ctrlList->controllers[i].w=-lim_w;

            //ctrlList->controllers[i].v*=0.7;
            //ctrlList->controllers[i].w*=0.7;
            //ctrlList->controllers[i].v=0;
            //ctrlList->controllers[i].w=0;
        }
#endif

        //invia input ai robot
        for (i=0;i<robList->numRob;i++) {
            float v=ctrlList->controllers[i].v;
            float w=ctrlList->controllers[i].w;
            //printf("%f\t%f\n",v,w);


            /*
            		c_t++;
            		printf("cont: %d",c_t);

            		if (c_t>20){
            			v=0;
            		}*/

            robList->robots[i].input_prev->val[0][0]=v;
            robList->robots[i].input_prev->val[1][0]=w;
            printf("\nV LIN %f\n",v);
            printf("W %f\n",w);
            //getchar();

#ifdef CON_ROBOT
            send_wifi_vel(sock[i],v,w);
#endif
        }

#ifdef FORM_CTRL
        //stampa log su file
#ifdef LOG
        flog1=fopen(LOG_FILE1,"a");
        //Le colonne sono: pos des x e y, pos attuale baricentro x e y, posizione attuale pt ctrl, input v e w
        fprintf(flog1,"%f\t",x_des_form);
        fprintf(flog1,"%f\t",y_des_form);
        fprintf(flog1,"%f\t",ctrlList->controllers[0].x_des);
        fprintf(flog1,"%f\t",ctrlList->controllers[0].y_des);
        fprintf(flog1,"%f\t",robList->robots[0].state->val[0][0]);
        fprintf(flog1,"%f\t",robList->robots[0].state->val[1][0]);
        fprintf(flog1,"%f\t",ctrlList->controllers[0].x_ctrl);
        fprintf(flog1,"%f\t",ctrlList->controllers[0].y_ctrl);
        fprintf(flog1,"%f\t",v);
        fprintf(flog1,"%f\n",w);

        fclose(flog1);

        //*
        flog2=fopen(LOG_FILE2,"a");
        //Le colonne sono: pos des x e y, pos attuale baricentro x e y, posizione attuale pt ctrl, input v e w
        fprintf(flog2,"%f\t",x_des_form);
        fprintf(flog2,"%f\t",y_des_form);
        fprintf(flog2,"%f\t",ctrlList->controllers[1].x_des);
        fprintf(flog2,"%f\t",ctrlList->controllers[1].y_des);
        fprintf(flog2,"%f\t",robList->robots[1].state->val[0][0]);
        fprintf(flog2,"%f\t",robList->robots[1].state->val[1][0]);
        fprintf(flog2,"%f\t",ctrlList->controllers[1].x_ctrl);
        fprintf(flog2,"%f\t",ctrlList->controllers[1].y_ctrl);
        fprintf(flog2,"%f\t",v);
        fprintf(flog2,"%f\n",w);

        fclose(flog2);
        //*/
#endif
#endif

#ifdef	SWARM_CTRL
        //stampa log su file
#ifdef LOG
        flog1=fopen(LOG_FILE1,"a");
        //Le colonne sono: pos attuale baricentro x e y, posizione attuale pt ctrl, input v e w
        fprintf(flog1,"%f\t",robList->robots[0].state->val[0][0]);
        fprintf(flog1,"%f\t",robList->robots[0].state->val[1][0]);
        fprintf(flog1,"%f\t",ctrlList->controllers[0].x_ctrl);
        fprintf(flog1,"%f\t",ctrlList->controllers[0].y_ctrl);
        fprintf(flog1,"%f\t",v);
        fprintf(flog1,"%f\n",w);
        fclose(flog1);

        //*
        flog2=fopen(LOG_FILE2,"a");
        //Le colonne sono: pos attuale baricentro x e y, posizione attuale pt ctrl, input v e w
        fprintf(flog2,"%f\t",robList->robots[1].state->val[0][0]);
        fprintf(flog2,"%f\t",robList->robots[1].state->val[1][0]);
        fprintf(flog2,"%f\t",ctrlList->controllers[1].x_ctrl);
        fprintf(flog2,"%f\t",ctrlList->controllers[1].y_ctrl);
        fprintf(flog2,"%f\t",v);
        fprintf(flog2,"%f\n",w);
        fclose(flog2);
        //*/
#endif
#endif

        //QUARTO TEMPO
        gettimeofday(&tm3,NULL);
        /*CALCOLO TEMPI ESECUZIONE
        //CALCOLO PERIODI
        //PRIMO
        if(tm1.tv_sec==tm0.tv_sec)
        	p0=(tm1.tv_usec-tm0.tv_usec)/(double)1000000;
        else
        	p0=(1000000-(tm1.tv_usec-tm0.tv_usec))/(double)1000000;

        //SECONDO
        if(tm2.tv_sec==tm1.tv_sec)
        	p1=(tm2.tv_usec-tm1.tv_usec)/(double)1000000;
        else
        	p1=(1000000-(tm2.tv_usec-tm1.tv_usec))/(double)1000000;

        //TERZO
        if(tm3.tv_sec==tm2.tv_sec)
        	p2=(tm3.tv_usec-tm2.tv_usec)/(double)1000000;
        else
        	p2=(1000000-(tm3.tv_usec-tm2.tv_usec))/(double)1000000;

        //TOTALE
        if(tm3.tv_sec==tm0.tv_sec)
        	pt=(tm3.tv_usec-tm0.tv_usec)/(double)1000000;
        else
        	pt=(1000000-(tm3.tv_usec-tm0.tv_usec))/(double)1000000;

        #ifdef TIME_CTRL
        ftime=fopen(TIME_FILE,"a");
        fprintf(ftime,"%f\t%f\t%f\t%f\n",pt,p0,p1,p2);
        fclose(ftime);
        #endif
        //*/
        /*
        //aggiorno formazione
        form->x_b=0;
        form->y_b=0;
        for(i=0;i<robList->numRob;i++){
        	//calcolo baricentro formazione
        	form->x_b=(i*form->x_b+robList->robots[i].state->val[0][0])/i+1;
        	form->y_b=(i*form->y_b+robList->robots[i].state->val[1][0])/i+1;

        	//calcolo punto controllato robot
        	(*ctrlList)->controllers[i].x_ctrl=(*robList)->robots[i].state->val[0][0]+(*ctrlList)->controllers[i].d*cosf((*robList)->robots[i].state->val[2][0]);
        	(*ctrlList)->controllers[i].y_ctrl=(*robList)->robots[i].state->val[1][0]+(*ctrlList)->controllers[i].d*sinf((*robList)->robots[i].state->val[2][0]);

        	//calcolo punto controllato formazione e punto desiderato
        	if((*robList)->robots[i].ip==ipP){
        		//punto controllato
        		(*form)->x_c=(*ctrlList)->controllers[i].x_ctrl;
        		(*form)->y_c=(*ctrlList)->controllers[i].y_ctrl;
        		//punto desiderato

        	}
        }
        (*form)->fi_c=atan2f((*form)->y_c-(*form)->y_b,(*form)->x_c-(*form)->x_b);




        for(i=0;i<ctrlList->numController;i++){


        }//*/
#endif

        //reset coordinate per iterazione successiva---togliere???
        for (i=0;i<robList->numRob;i++) {
            cnst(HUGE_VALF,coord);
        }
        /*
        printf("INVIO VEL\n");
        if (conta<30){
        	//printf("ECCOLO!!\n");
        	//getchar();
        		v=10;
        		w=0;
        		robList->robots[0].input_prev->val[0][0]=v;
        		robList->robots[0].input_prev->val[1][0]=w;
        		send_wifi_vel(sock[0],v,w);
        		conta++;
        }

        if (conta>=30 && conta<40){
        		//printf("AAAAAAAAAAA QUI!!!!%d\n",conta);
        		//getchar();
        		v=0;
        		w=M_PI/6;
        		robList->robots[0].input_prev->val[0][0]=v;
        		robList->robots[0].input_prev->val[1][0]=w;
        		send_wifi_vel(sock[0],v,w);
        		conta++;
        }

        if (conta>=40 && conta<90){
        	//printf("ECCOLO!!\n");
        	//getchar();
        		v=10;
        		w=0;
        		robList->robots[0].input_prev->val[0][0]=v;
        		robList->robots[0].input_prev->val[1][0]=w;
        		send_wifi_vel(sock[0],v,w);
        		conta++;
        }

        if (conta>=90 && conta<100){
        		//printf("AAAAAAAAAAA QUI!!!!%d\n",conta);
        		//getchar();
        		v=0;
        		w=-M_PI/6;
        		robList->robots[0].input_prev->val[0][0]=v;
        		robList->robots[0].input_prev->val[1][0]=w;
        		send_wifi_vel(sock[0],v,w);
        		conta++;
        }
        if (conta>=100){
        	//printf("ECCOLO!!\n");
        	//getchar();
        		v=10;
        		w=0;
        		robList->robots[0].input_prev->val[0][0]=v;
        		robList->robots[0].input_prev->val[1][0]=w;
        		send_wifi_vel(sock[0],v,w);
        		conta++;
        }
        //*/
//if(conta==39)
//	conta=0;
//getchar();
//float v,w;
//for(j=0;j<robList->numRob;j++){
//	printf("INSERIRE VELOCITA' ROBOT %d")

//getchar();
//send_wifi_msg(sock[j],MSG_STOP);
//printf("\nMSG STOP %d\n",MSG_STOP);
//getchar();
    }
}
//========================================================================

/*
void prova(colorBlobList *colorBList){
printf("Prova: %f\n", colorBList->blobs[0].centroidX);
printf("Prova: %f\n", colorBList->blobs[0].centroidY);
printf("Prova: %d\n", colorBList->numBlob);
}

void prova(markerList *m1List){
printf("Prova: %f\n", m1List->markers[0].x);
}
*/
void prova(coupleList *cplList) {
    printf("Prova: %f\n",cplList->cpl[0].x1);
}

int main(int argc, char* argv[]) {
#ifdef PRINT
    printf("INIT\n");
#endif
    main_init();
    setup_termination();
    pthread_attr_t attr_main;
    pthread_attr_init(&attr_main);
    pthread_create(&thread_main, &attr_main, &tf_main, NULL);
    pthread_create(&thread_kinect, &attr_main, &tf_kinect, NULL);
    pthread_join(thread_main, NULL);
    return 0;
}
//=========================================================================================================
