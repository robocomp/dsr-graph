#include "/home/pbustos/software/darknet/include/darknet.h"	
//#include "/home/pbustos/software/darknet/src/box.h"	
//#include "/home/pbustos/software/darknet/src/network.h"	


static network *net;
static clock_t time1;
static float nms;
static int demo_frame = 3;
static int demo_index = 0;
static float **predictions;
static float *avg;
static int demo_total = 0;

// int size_network(network *net)
// {
//     int i;
//     int count = 0;
//     for(i = 0; i < net->n; ++i){
//         layer l = net->layers[i];
//         if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
//             count += l.outputs;
//         }
//     }
//     return count;
// }

// void remember_network(network *net)
// {
//     int i;
//     int count = 0;
//     for(i = 0; i < net->n; ++i){
//         layer l = net->layers[i];
//         if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
//             memcpy(predictions[demo_index] + count, net->layers[i].output, sizeof(float) * l.outputs);
//             count += l.outputs;
//         }
//     }
// }
// 
// void avg_predictions(network *net)
// {
//     int i, j;
//     int count = 0;
//     fill_cpu(demo_total, 0, avg, 1);
// 	for(j = 0; j < demo_frame; ++j){
//         axpy_cpu(demo_total, 1./demo_frame, predictions[j], 1, avg, 1);
//     }
//     for(i = 0; i < net->n; ++i){
//         layer l = net->layers[i];
//         if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
//             memcpy(l.output, avg + count, sizeof(float) * l.outputs);
//             count += l.outputs;
//         }
//     }
// }


void init_detector(char  *datacfg, char *cfgfile, char *weightfile, char *filename, float thresh, float hier_thresh, char *outfile, int fullscreen, char **names) 
{
	//cuda_set_device(0);
	//options = read_data_cfg(datacfg);
	names = get_labels(filename);
	//alphabet = load_alphabet();
	net = load_network(cfgfile, weightfile, 0);
	set_batch_network(net, 1);
	demo_total = size_network(net);
	predictions = calloc(demo_frame, sizeof(float*));
	for (int i = 0; i < demo_frame; ++i)
				predictions[i] = calloc(demo_total, sizeof(float));
	avg = calloc(demo_total, sizeof(float));
	srand(2222222);
	nms=.4;
}

detection* detector(float thresh, float hier_thresh, image *im, int *numboxes)
{
	time1=clock();

	//image sized = letterbox_image(im, net->w, net->h);
	//printf("net %d %d \n", net->w, net->h);
	
	//printf("Letterbox elapsed %f mseconds.\n", sec(clock()-time1)*1000);
	//time1=clock();
	
	layer l = net->layers[net->n-1];

	network_predict(net, im->data);
	remember_network(net);
	detection *dets  = 0;
	avg_predictions(net);
	dets = get_network_boxes(net, im->w, im->h, thresh, hier_thresh, 0, 1, numboxes);
    
	//printf("Test-Detector: Network-predict elapsed in %f mseconds.\n",sec(clock()-time1)*1000);
	//time1=clock();
	
	const float solapamiento = 0.3;
	//printf("antes clases %d numboxes %d \n", 20, *numboxes);
	do_nms_obj(dets, *numboxes, l.classes, solapamiento);
	//printf("despues clases %d numboxes %d \n", 20, *numboxes);
	
	//do_nms_sort(dets, *numboxes, l.classes, solapamiento);
	
	//free_detections(dets, *numboxes);
	
	return dets;
}

