#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <windows.h>

#include "utils/helpers.hpp"
#include "utils/command-line-arguments.hpp"
#include "utils/points.hpp"
#include "tracker/FaceTracker.hpp"

#include "gl/glew.h"
#include <gl/glut.h>
#include "../packages/glm.0.9.6.3/build/native/include/glm/glm.hpp"
#include "../packages/glm.0.9.6.3/build/native/include/glm/gtc/matrix_transform.hpp"
#include "../packages/glfw.3.0.4.3/build/native/include/GLFW/glfw3.h"
#include <gl/GL.h>

#include <algorithm>
#include <ctime>

#include "common.h"
#include "model.h"

using namespace FACETRACKER;
using namespace cv;
using namespace std;
using namespace glm;

double dWidth;
double dHeight;

float Ytop;
float Ybot;
float minDY;
float maxDY;

VideoCapture* cap;

struct Configuration
{
	double wait_time;
	std::string model_pathname;
	std::string params_pathname;
	int tracking_threshold;
	std::string window_title;
	bool verbose;
	bool save_3d_points;

	int circle_radius;
	int circle_thickness;
	int circle_linetype;
	int circle_shift;
};

Configuration cfg;
FaceTracker * tracker;
FaceTrackerParams *tracker_params;

GLFWwindow* window;
Model *model = NULL;

GLuint programID;
GLuint MatrixID;
GLuint ViewMatrixID;
GLuint ModelMatrixID;
GLuint open_wID;
GLuint Texture;
GLuint TextureID;
GLuint LightID;

vector<GLfloat> g_vertex_buffer_data;
vector<GLfloat> g_vertex_buffer_data_open;
vector<GLfloat> g_uv_buffer_data;
vector<GLfloat> g_normal_buffer_data;

GLfloat open_w;

GLuint vertexbuffer;
GLuint uvbuffer;
GLuint normalbuffer;
GLuint vertexopenbuffer;

cv::Mat
compute_pose_image(const Pose &pose, int height, int width)
{
	cv::Mat_<cv::Vec<uint8_t, 3> > rv = cv::Mat_<cv::Vec<uint8_t, 3> >::zeros(height, width);
	cv::Mat_<double> axes = pose_axes(pose);
	cv::Mat_<double> scaling = cv::Mat_<double>::eye(3, 3);

	for (int i = 0; i < axes.cols; i++) {
		axes(0, i) = -0.5*double(width)*(axes(0, i) - 1);
		axes(1, i) = -0.5*double(height)*(axes(1, i) - 1);
	}

	cv::Point centre(width / 2, height / 2);
	// pitch
	cv::line(rv, centre, cv::Point(axes(0, 0), axes(1, 0)), cv::Scalar(255, 0, 0));
	// yaw
	cv::line(rv, centre, cv::Point(axes(0, 1), axes(1, 1)), cv::Scalar(0, 255, 0));
	// roll
	cv::line(rv, centre, cv::Point(axes(0, 2), axes(1, 2)), cv::Scalar(0, 0, 255));

	return rv;
}


void
display_data(const Configuration &cfg,
const cv::Mat &image,
const std::vector<cv::Point_<double> > &points,
const Pose &pose)
{

	cv::Scalar colour;
	if (image.type() == cv::DataType<uint8_t>::type)
		colour = cv::Scalar(255);
	else if (image.type() == cv::DataType<cv::Vec<uint8_t, 3> >::type)
		colour = cv::Scalar(0, 0, 255);
	else
		colour = cv::Scalar(255);

	cv::Mat displayed_image;
	if (image.type() == cv::DataType<cv::Vec<uint8_t, 3> >::type)
		displayed_image = image.clone();
	else if (image.type() == cv::DataType<uint8_t>::type)
		cv::cvtColor(image, displayed_image, CV_GRAY2BGR);
	else
		throw make_runtime_error("Unsupported camera image type for display_data function.");

	for (size_t i = 0; i < points.size(); i++) {
		cv::circle(displayed_image, points[i], cfg.circle_radius, colour, cfg.circle_thickness, cfg.circle_linetype, cfg.circle_shift);
	}

	int pose_image_height = 100;
	int pose_image_width = 100;
	cv::Mat pose_image = compute_pose_image(pose, pose_image_height, pose_image_width);
	for (int i = 0; i < pose_image_height; i++) {
		for (int j = 0; j < pose_image_width; j++) {
			displayed_image.at<cv::Vec<uint8_t, 3> >(displayed_image.rows - pose_image_height + i,
				displayed_image.cols - pose_image_width + j)

				= pose_image.at<cv::Vec<uint8_t, 3> >(i, j);
		}
	}

	imshow("MyVideo", displayed_image);

}

void detect()
{
	Mat frame;

	bool bSuccess = cap->read(frame); // read a new frame from video

	if (!bSuccess) //if not success, break loop
	{
		cout << "Cannot read a frame from video stream" << endl;
		return;
	}

	cv::Mat image;
	cv::cvtColor(frame, image, CV_BGR2GRAY);

	int result = tracker->NewFrame(image, tracker_params);

	std::vector<cv::Point_<double> > shape;
	std::vector<cv::Point3_<double> > shape3;
	Pose pose;

	if (result >= cfg.tracking_threshold) {
		shape = tracker->getShape();
		shape3 = tracker->get3DShape();
		pose = tracker->getPose();

		//vector<float> mouth_pointsX;
		vector<float> mouth_pointsY;
		for (int i = 48; i < shape.size(); i++)
		{
			//mouth_pointsX.push_back((float)shape[i].x);
			mouth_pointsY.push_back((float)shape[i].y);
		}

		//float Xleft = *min_element(mouth_pointsX.begin(), mouth_pointsX.end());
		//float Xright = *max_element(mouth_pointsX.begin(), mouth_pointsX.end());
		double tmp1 = *min_element(mouth_pointsY.begin(), mouth_pointsY.end());
		double tmp2 = *max_element(mouth_pointsY.begin(), mouth_pointsY.end());
		if (abs(tmp2 - tmp1) >= 10.0f)
		{
			Ytop = tmp1;
			Ybot = tmp2;
		}

		//cout << Ytop << " " << Ybot << endl;
	}

	display_data(cfg, frame, shape, pose);

}

GLvoid render(GLvoid)
{
	open_w = 0.0f;

	do{
		detect();

		open_w = (Ybot - Ytop - minDY)/(maxDY - minDY);
		open_w = std::min(std::max(0.0f, open_w),1.0f);

		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Use our shader
		glUseProgram(programID);
		
		
		mat4 ProjectionMatrix = glm::perspective(
			45.0f,         // The horizontal Field of View, in degrees : the amount of "zoom". Think "camera lens". Usually between 90° (extra wide) and 30° (quite zoomed in)
			4.0f / 3.0f, // Aspect Ratio. Depends on the size of your window. Notice that 4/3 == 800/600 == 1280/960, sounds familiar ?
			0.1f,        // Near clipping plane. Keep as big as possible, or you'll get precision issues.
			100.0f       // Far clipping plane. Keep as little as possible.
			);
		mat4 ViewMatrix = lookAt(
			vec3(0.0f, 0.2f, 3.0f), // Camera is at (4,3,3), in World Space
			vec3(0, 0, 0), // and looks at the origin
			vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
			);
		/*
		mat4 ProjectionMatrix = glm::perspective(
			45.0f,         // The horizontal Field of View, in degrees : the amount of "zoom". Think "camera lens". Usually between 90° (extra wide) and 30° (quite zoomed in)
			4.0f / 3.0f, // Aspect Ratio. Depends on the size of your window. Notice that 4/3 == 800/600 == 1280/960, sounds familiar ?
			0.1f,        // Near clipping plane. Keep as big as possible, or you'll get precision issues.
			100.0f       // Far clipping plane. Keep as little as possible.
			);
		mat4 ViewMatrix = lookAt(
			vec3(0.0f, 4.0f, 25.0f), // Camera is at (4,3,3), in World Space
			vec3(0, 2, 0), // and looks at the origin
			vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
			);
		*/

		mat4 ModelMatrix = mat4(1.0f);
		mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

		// Send our transformation to the currently bound shader, 
		// in the "MVP" uniform
		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
		glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
		glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);

		vec3 lightPos = vec3(4, 4, 4);
		glUniform3f(LightID, lightPos.x, lightPos.y, lightPos.z);

		glUniform1f(open_wID, open_w);


		// Bind our texture in Texture Unit 0
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, Texture);
		// Set our "myTextureSampler" sampler to user Texture Unit 0
		glUniform1i(TextureID, 0);

		// 1rst attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
		glVertexAttribPointer(
			0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
			3,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
			);
		// 2nd attribute buffer : UVs
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
		glVertexAttribPointer(
			1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
			2,                                // size : U+V => 2
			GL_FLOAT,                         // type
			GL_FALSE,                         // normalized?
			0,                                // stride
			(void*)0                          // array buffer offset
			);

		// 3rd attribute buffer : normals
		glEnableVertexAttribArray(2);
		glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
		glVertexAttribPointer(
			2,                                // attribute
			3,                                // size
			GL_FLOAT,                         // type
			GL_FALSE,                         // normalized?
			0,                                // stride
			(void*)0                          // array buffer offset
			);

		//open verts
		glEnableVertexAttribArray(3);
		glBindBuffer(GL_ARRAY_BUFFER, vertexopenbuffer);
		glVertexAttribPointer(
			3,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
			3,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
			);

		// Draw the triangle !
		glDrawArrays(GL_TRIANGLES, 0, g_vertex_buffer_data.size()*3); // 12*3 indices starting at 0 -> 12 triangles -> 6 squares

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
		glDisableVertexAttribArray(2);
		glDisableVertexAttribArray(3);

		/*
		cv::Mat img(480, 640, CV_8UC3);
		img.create(480, 640, CV_8UC3);
		//use fast 4-byte alignment (default anyway) if possible
		glPixelStorei(GL_PACK_ALIGNMENT, (img.step & 3) ? 1 : 4);
		//set length of one complete row in destination data (doesn't need to equal img.cols)
		glPixelStorei(GL_PACK_ROW_LENGTH, img.step / img.elemSize());
		glReadPixels(0, 0, img.cols, img.rows, GL_BGR, GL_UNSIGNED_BYTE, img.data);
		cv::Mat flipped(480, 640, CV_8UC3);
		cv::flip(img, flipped, 0);
		namedWindow("Display window", WINDOW_AUTOSIZE);// Create a window for display.
		imshow("Display window", flipped);
		*/

		// Swap buffers
		glfwSwapBuffers(window);
		
		glfwPollEvents();

	} // Check if the ESC key was pressed or the window was closed
	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS && glfwWindowShouldClose(window) == 0);
}

void reloadTracker()
{
	delete tracker;
	delete tracker_params;
	Ybot = (float)dHeight;
	Ytop = 0.0f;

	minDY = (float)dHeight;
	maxDY = 0.0f;
	tracker = LoadFaceTracker(cfg.model_pathname.c_str());
	tracker_params = LoadFaceTrackerParams(cfg.params_pathname.c_str());
}

int main(int argc, char **argv)
{	
	cap = new VideoCapture(0); // open the video camera no. 0

	if (!cap->isOpened())  // if not success, exit program
	{
		cout << "Cannot open the video cam" << endl;
		return -1;
	}

	model = new Model("src/african_head.obj", "src/african_head_open.obj");
	//model = new Model("src/jason_head.obj", "src/jason_head_open.obj");

	double dWidth = cap->get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = cap->get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

	cout << "Frame size : " << dWidth << " x " << dHeight << endl;

	namedWindow("MyVideo", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

	cfg.wait_time = 0;
	cfg.model_pathname = DefaultFaceTrackerModelPathname();
	cfg.params_pathname = DefaultFaceTrackerParamsPathname();
	cfg.tracking_threshold = 0;//5
	cfg.window_title = "CSIRO Face Fit";
	cfg.verbose = false;
	cfg.circle_radius = 2;
	cfg.circle_thickness = 1;
	cfg.circle_linetype = 8;
	cfg.circle_shift = 0;
	cfg.save_3d_points = false;
	cfg.wait_time = 0;

	tracker = LoadFaceTracker(cfg.model_pathname.c_str());
	tracker_params = LoadFaceTrackerParams(cfg.params_pathname.c_str());


	minDY = (float)dHeight;
	maxDY = 0.0f;

	for (;;)
	{
		detect();
		minDY = std::min(std::max(abs(Ybot - Ytop),10.0f), minDY);
		maxDY = std::max(abs(Ybot - Ytop), maxDY);
		cout << "/// " << minDY << " " << maxDY << endl;

		int k = waitKey(100);
		if (k >= 0)
		{
			if (k == 32)
				reloadTracker();
			else break;
		} 
	}


	//////////

	if (!glfwInit())
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Open a window and create its OpenGL context
	window = glfwCreateWindow(dWidth, dHeight, "Facial Modeling", NULL, NULL);
	if (window == NULL){
		fprintf(stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n");
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		return -1;
	}

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

	// Dark blue background
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS);

	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);

	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	// Create and compile our GLSL program from the shaders
	programID = LoadShaders("vs.vertexshader", "fs.fragmentshader");


	MatrixID = glGetUniformLocation(programID, "MVP");
	ViewMatrixID = glGetUniformLocation(programID, "V");
	ModelMatrixID = glGetUniformLocation(programID, "M");
	open_wID = glGetUniformLocation(programID, "open_w");
	LightID = glGetUniformLocation(programID, "lightposition");
	
	// Create one OpenGL texture
	glGenTextures(1, &Texture);

	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, Texture);

	// Give the image to OpenGL
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, model->diffusemap().get_width(), model->diffusemap().get_height(), 0, GL_BGR, GL_UNSIGNED_BYTE, model->diffusemap().getdata());

	// ... nice trilinear filtering.
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glGenerateMipmap(GL_TEXTURE_2D);

	///////////

	// Get a handle for our "myTextureSampler" uniform
	TextureID = glGetUniformLocation(programID, "diffuse");

	for (int i = 0; i < model->nfaces(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				g_vertex_buffer_data.push_back(model->vert(i, j)[k]);
				g_vertex_buffer_data_open.push_back(model->vertopen(i, j)[k]);
			}
			for (int k = 0; k < 2; k++)
				g_uv_buffer_data.push_back(model->uv(i, j)[k]);
			for (int k = 0; k < 3; k++)
				g_normal_buffer_data.push_back(model->normal(i, j)[k]);
		}
	}

	GLfloat *a = g_vertex_buffer_data.data();
	GLfloat *b = g_uv_buffer_data.data();
	GLfloat *c = g_normal_buffer_data.data();
	GLfloat *d = g_vertex_buffer_data_open.data();

	
	// This will identify our vertex buffer
	// Generate 1 buffer, put the resulting identifier in vertexbuffer
	glGenBuffers(1, &vertexbuffer);
	// The following commands will talk about our 'vertexbuffer' buffer
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	// Give our vertices to OpenGL.
	glBufferData(GL_ARRAY_BUFFER, sizeof(a)*g_vertex_buffer_data.size(), a, GL_STATIC_DRAW);

	glGenBuffers(1, &uvbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(b)*g_uv_buffer_data.size(), b, GL_STATIC_DRAW);

	
	glGenBuffers(1, &normalbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(c)*g_normal_buffer_data.size(), c, GL_STATIC_DRAW);

	glGenBuffers(1, &vertexopenbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexopenbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(d)*g_vertex_buffer_data_open.size(), d, GL_STATIC_DRAW);
	

	// Get a handle for our "LightPosition" uniform
	glUseProgram(programID);

	//
	render();
	//

	// Cleanup VBO
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteBuffers(1, &uvbuffer);
	glDeleteBuffers(1, &normalbuffer);
	glDeleteBuffers(1, &vertexopenbuffer);
	glDeleteProgram(programID);
	glDeleteTextures(1, &TextureID);
	glDeleteVertexArrays(1, &VertexArrayID);
	

	// Close OpenGL window and terminate GLFW
	glfwTerminate();

	delete model;

	delete tracker;
	delete tracker_params;

	return 0;
}