#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class CameraNode {
public:
    CameraNode(int video_id) {
        // Inicializa el objeto de captura de OpenCV
        
        cap.open(video_id); // Cambiar 0 por el índice de la  cámara si no funciona
        if (!cap.isOpened()) {
            ROS_ERROR("No se pudo abrir la cámara.");	//Si la camara no se abre, envia un mensaje de error
            ros::shutdown();
        }
	//Se crea un publicador que envia mensajes de tipo sensor_msgs::Image al topic camera/image_raw de tamaño 1 paa la cola de mensajes
        image_pub = nh.advertise<sensor_msgs::Image>("camera/image_raw", 1);
    }
    //Capturamos una imagen y la publicamos
    void publishImage() {
        cv::Mat frame;
        cap >> frame; // Captura un nuevo frame
	//Generamos el mensaje de imagen con los datos que nos ofrece
    
        if (!frame.empty()) {
            cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

            // ========================
            // === MEJORA DE CONTRASTE (CLAHE) ===
            // ========================
            // Convertir de RGB a LAB (donde el canal L es el de luminosidad)
            cv::Mat lab_image;
            cv::cvtColor(frame, lab_image, cv::COLOR_RGB2Lab);

            // Separar los canales LAB
            std::vector<cv::Mat> lab_planes(3);
            cv::split(lab_image, lab_planes);

            // Aplicar CLAHE al canal de luminosidad (L)
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
            clahe->setClipLimit(4.0); // Puedes ajustar este valor
            clahe->apply(lab_planes[0], lab_planes[0]);

            // Volver a unir los canales LAB
            cv::merge(lab_planes, lab_image);

            // Convertir de LAB a RGB de nuevo
            cv::cvtColor(lab_image, frame, cv::COLOR_Lab2RGB);



            cv_bridge::CvImage img_msg;

            //sensor_msgs::Image img_msg;
            img_msg.header.stamp = ros::Time::now();	//Marca de tiempo
            //img_msg.height = frame.rows;	//Altura imagen
            //img_msg.width = frame.cols;		//Ancho imagen
            img_msg.encoding = sensor_msgs::image_encodings::RGB8;		//Codificacion BGR8
            //img_msg.is_bigendian = 0;		//No big-endian --> little endian
            //img_msg.step = frame.step[0];	//Numero de bytes por fila
            //img_msg.data.assign(frame.datastart, frame.dataend);	//Copia todos los datos de la imagen desde frame a img_msg
            img_msg.image = frame;
            image_pub.publish(img_msg.toImageMsg()); // Publica la imagen en el topic
        }
    }
private:
    cv::VideoCapture cap;
    ros::NodeHandle nh;
    ros::Publisher image_pub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_node");//Inicializamos el nodo de ROS
    int video_id = std::stoi(argv[1]);
    CameraNode camera_node(video_id);
    ros::Rate loop_rate(30); // Publica a 30 Hz
    //Buclke principal que se ejecuta mientras el nodo este activo
    while (ros::ok()) {
        camera_node.publishImage();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
