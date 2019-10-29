
#include "gl_program.h"

static void get_rendering_params(PICAM360CAPTURE_T *state, FRAME_T *frame, VECTOR4D_T view_quat, RENDERING_PARAMS_T *params);

static void init_ogl(PICAM360CAPTURE_T *state);
static void init_textures(PICAM360CAPTURE_T *state);

static void init_model_proj(PICAM360CAPTURE_T *state);
static void redraw_render_texture(PICAM360CAPTURE_T *state, FRAME_T *frame, RENDERER_T *renderer, VECTOR4D_T view_quat);
static void redraw_scene(PICAM360CAPTURE_T *state, FRAME_T *frame, MODEL_T *model);
static void redraw_info(PICAM360CAPTURE_T *state, FRAME_T *frame);

static bool lg_debug_dump = false;
static int lg_debug_dump_num = 0;
static int lg_debug_dump_fd = -1;


int board_mesh(int num_of_steps, GLuint *vbo_out, GLuint *n_out, GLuint *vao_out) {
	GLuint vbo;

	int n = 2 * (num_of_steps + 1) * num_of_steps;
	float points[4 * n];

	float start_x = 0.0f;
	float start_y = 0.0f;

	float end_x = 1.0f;
	float end_y = 1.0f;

	float step_x = (end_x - start_x) / num_of_steps;
	float step_y = (end_y - start_y) / num_of_steps;

	int idx = 0;
	int i, j;
	for (i = 0; i < num_of_steps; i++) {	//x
		for (j = 0; j <= num_of_steps; j++) {	//y
			{
				float x = start_x + step_x * i;
				float y = start_y + step_y * j;
				float z = 1.0;
				points[idx++] = x;
				points[idx++] = y;
				points[idx++] = z;
				points[idx++] = 1.0;
				//printf("x=%f,y=%f,z=%f,w=%f\n", points[idx - 4],
				//		points[idx - 3], points[idx - 2], points[idx - 1]);
			}
			{
				float x = start_x + step_x * (i + 1);
				float y = start_y + step_y * j;
				float z = 1.0;
				points[idx++] = x;
				points[idx++] = y;
				points[idx++] = z;
				points[idx++] = 1.0;
				//printf("x=%f,y=%f,z=%f,w=%f\n", points[idx - 4],
				//		points[idx - 3], points[idx - 2], points[idx - 1]);
			}
		}
	}

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * n, points, GL_STATIC_DRAW);

#ifdef USE_GLES
#else
	GLuint vao;
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glBindVertexArray(0);

	if (vao_out != NULL)
		*vao_out = vao;
#endif

	if (vbo_out != NULL)
		*vbo_out = vbo;
	if (n_out != NULL)
		*n_out = n;

	return 0;
}

bool destroy_vstream(VOSTREAM_T *stream) {
	printf("delete_frame id=%d\n", frame->id);

	if (frame->befor_deleted_callback) {
		frame->befor_deleted_callback(state, frame);
	}

	if (frame->framebuffer) {
		glDeleteFramebuffers(1, &frame->framebuffer);
		frame->framebuffer = 0;
	}
	if (frame->texture) {
		glDeleteTextures(1, &frame->texture);
		frame->texture = 0;
	}
	if (frame->img_buff) {
		free(frame->img_buff);
		frame->img_buff = NULL;
	}
	if (frame->view_mpu) {
		frame->view_mpu->release(frame->view_mpu);
		frame->view_mpu = NULL;
	}

	if (frame->encoder) { //stop record
		frame->encoder->release(frame->encoder);
		frame->encoder = NULL;
	}

	if (frame->output_fd >= 0) {
		close(frame->output_fd);
		frame->output_fd = -1;
	}

	free(frame);

	return true;
}
FRAME_T *create_vostream(PICAM360CAPTURE_T *state, const char *_buff) {
	GLenum err;
	int opt;
	int render_width = 512;
	int render_height = 512;
	FRAME_T *frame = malloc(sizeof(FRAME_T));
	memset(frame, 0, sizeof(FRAME_T));
	frame->id = ++state->last_vostream_id;//frame id should start from 1
	frame->output_mode = OUTPUT_MODE_NONE;
	frame->output_type = OUTPUT_TYPE_NONE;
	frame->output_fd = -1;
	frame->fov = 120;

	optind = 1; // reset getopt
	while ((opt = getopt(argc, argv, "w:h:m:o:s:v:f:k:")) != -1) {
		switch (opt) {
		case 'w':
			sscanf(optarg, "%d", &render_width);
			break;
		case 'h':
			sscanf(optarg, "%d", &render_height);
			break;
		case 'm':
			//TODO
			//frame->renderer = get_renderer(optarg);
			if (frame->renderer == NULL) {
				printf("%s renderer is not existing.\n", optarg);
			}
			break;
		case 'o':
			frame->output_mode = OUTPUT_MODE_VIDEO;
			strncpy(frame->output_filepath, optarg, sizeof(frame->output_filepath));
			break;
		case 'v':
			for (int i = 0; state->mpu_factories[i] != NULL; i++) {
				if (strncmp(state->mpu_factories[i]->name, optarg, 64) == 0) {
					state->mpu_factories[i]->create_mpu(state->mpu_factories[i]->user_data, &frame->view_mpu);
				}
			}
			break;
		case 's':
			frame->output_mode = OUTPUT_MODE_STREAM;
			for (int i = 0; state->streamer_factories[i] != NULL; i++) {
				if (strncmp(state->streamer_factories[i]->name, optarg, sizeof(frame->encoder)) == 0) {
					state->streamer_factories[i]->create_streamer(state->streamer_factories[i]->user_data, &frame->encoder);
				}
			}
			if (frame->encoder == NULL) {
				printf("%s is not supported\n", optarg);
			}
			//h264 or mjpeg
			if (strcasecmp(optarg, "h265") == 0) {
				frame->output_type = OUTPUT_TYPE_H265;
			} else if (strcasecmp(optarg, "h264") == 0) {
				frame->output_type = OUTPUT_TYPE_H264;
			}  else if (strcasecmp(optarg, "i420") == 0) {
				frame->output_type = OUTPUT_TYPE_I420;
			} else {
				frame->output_type = OUTPUT_TYPE_MJPEG;
			}
			break;
		case 'f':
			sscanf(optarg, "%f", &frame->fps);
			break;
		case 'k':
			sscanf(optarg, "%f", &frame->kbps);
			break;
		default:
			break;
		}
	}

	if (frame->view_mpu == NULL) {
		for (int i = 0; state->mpu_factories[i] != NULL; i++) {
			if (strncmp(state->mpu_factories[i]->name, state->default_view_coordinate_mode, 64) == 0) {
				state->mpu_factories[i]->create_mpu(state->mpu_factories[i]->user_data, &frame->view_mpu);
			}
		}
	}

	if (render_width > 2048) {
		frame->double_size = true;
		frame->width = render_width / 2;
		frame->height = render_height;
	} else {
		frame->width = render_width;
		frame->height = render_height;
	}

	state->plugin_host.lock_texture();
	{
		//texture rendering
		glGenFramebuffers(1, &frame->framebuffer);

		glGenTextures(1, &frame->texture);
		glBindTexture(GL_TEXTURE_2D, frame->texture);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame->width, frame->height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
		if ((err = glGetError()) != GL_NO_ERROR) {
#ifdef USE_GLES
			printf("glTexImage2D failed. Could not allocate texture buffer.\n");
#else
			printf("glTexImage2D failed. Could not allocate texture buffer.\n %s\n", gluErrorString(err));
#endif
		}
		glBindTexture(GL_TEXTURE_2D, 0);

		glBindFramebuffer(GL_FRAMEBUFFER, frame->framebuffer);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, frame->texture, 0);
		if ((err = glGetError()) != GL_NO_ERROR) {
#ifdef USE_GLES
			printf("glFramebufferTexture2D failed. Could not allocate framebuffer.\n");
#else
			printf("glFramebufferTexture2D failed. Could not allocate framebuffer. %s\n", gluErrorString(err));
#endif
		}

		// Set background color and clear buffers
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}
	state->plugin_host.unlock_texture();

	//buffer memory
	if (frame->double_size) {
		int size = frame->width * frame->height * 3;
		frame->img_buff = (unsigned char*) malloc(size * 2);
	} else {
		int size = frame->width * frame->height * 3;
		frame->img_buff = (unsigned char*) malloc(size);
	}

	printf("create_vostream id=%d\n", frame->id);

	return frame;
}
static void exit_func(void)
// Function to be passed to atexit().
{
#ifdef USE_GLES
	deinit_textures(state);

	// clear screen
	glClear(GL_COLOR_BUFFER_BIT);
	eglSwapBuffers(state->display, state->surface);

	// Release OpenGL resources
	eglMakeCurrent(state->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
	eglDestroySurface(state->display, state->surface);
	eglDestroyContext(state->display, state->context);
	eglTerminate(state->display);
#endif
	printf("\npicam360-capture closed\n");
} // exit_func()

static void get_rendering_params(PICAM360CAPTURE_T *state, FRAME_T *frame, VECTOR4D_T view_quat, RENDERING_PARAMS_T *params) {
	{
		params->stereo = frame->stereo;
		params->fov = frame->fov;
		params->active_cam = state->active_cam;
		params->num_of_cam = state->num_of_cam;
	}
	{ //cam_attitude //depth axis is z, vertical asis is y
		float view_matrix[16];
		float north_matrix[16];
		float world_matrix[16];

		{ // Rv : view
			mat4_identity(view_matrix);
			mat4_fromQuat(view_matrix, view_quat.ary);
			mat4_invert(view_matrix, view_matrix);
		}

		{ // Rn : north
			mat4_identity(north_matrix);
			float north = state->plugin_host.get_view_north();
			mat4_rotateY(north_matrix, north_matrix, north * M_PI / 180);
		}

		{ // Rw : view coodinate to world coodinate and view heading to ground initially
			mat4_identity(world_matrix);
			mat4_rotateX(world_matrix, world_matrix, -M_PI / 2);
		}

		for (int i = 0; i < state->num_of_cam; i++) {
			float *unif_matrix = params->cam_attitude[i];
			float cam_matrix[16];
			{ // Rc : cam orientation
				mat4_identity(cam_matrix);
				if (state->camera_coordinate_from_device) {
					mat4_fromQuat(cam_matrix, state->camera_quaternion[i].ary);
				} else {
					//euler Y(yaw)X(pitch)Z(roll)
					mat4_rotateZ(cam_matrix, cam_matrix, state->camera_roll);
					mat4_rotateX(cam_matrix, cam_matrix, state->camera_pitch);
					mat4_rotateY(cam_matrix, cam_matrix, state->camera_yaw);
				}
			}

			{ // Rco : cam offset  //euler Y(yaw)X(pitch)Z(roll)
				float *cam_offset_matrix = params->cam_offset_matrix[i];
				mat4_identity(cam_offset_matrix);
				mat4_rotateZ(cam_offset_matrix, cam_offset_matrix, state->options.cam_offset_roll[i]);
				mat4_rotateX(cam_offset_matrix, cam_offset_matrix, state->options.cam_offset_pitch[i]);
				mat4_rotateY(cam_offset_matrix, cam_offset_matrix, state->options.cam_offset_yaw[i]);
				mat4_invert(cam_offset_matrix, cam_offset_matrix);
				mat4_multiply(cam_matrix, cam_matrix, cam_offset_matrix); // Rc'=RcoRc
			}

			{ //RcRv(Rc^-1)RcRw
				mat4_identity(unif_matrix);
				mat4_multiply(unif_matrix, unif_matrix, world_matrix); // Rw
				mat4_multiply(unif_matrix, unif_matrix, view_matrix); // RvRw
				//mat4_multiply(unif_matrix, unif_matrix, north_matrix); // RnRvRw
				mat4_multiply(unif_matrix, unif_matrix, cam_matrix); // RcRnRvRw
			}
			mat4_transpose(unif_matrix, unif_matrix); // this mat4 library is row primary, opengl is column primary
//			{
//				//normalize det = 1
//				float det = mat4_determinant(unif_matrix);
//				float det_4 = pow(det, 0.25);
//				for (int i = 0; i < 16; i++) {
//					unif_matrix[i] / det_4;
//				}
//				//printf("det=%f, %f, %f\n", det, pow(det, 0.25), mat4_determinant(unif_matrix));
//			}
		}
	}
	{ //cam_options
		for (int i = 0; i < state->num_of_cam; i++) {
			params->cam_offset_x[i] = state->options.cam_offset_x[i];
			params->cam_offset_y[i] = state->options.cam_offset_y[i];
			params->cam_horizon_r[i] = state->options.cam_horizon_r[i];
			params->cam_aov[i] = state->options.cam_aov[i];
			if (state->options.config_ex_enabled) {
				params->cam_offset_x[i] += state->options.cam_offset_x_ex[i];
				params->cam_offset_y[i] += state->options.cam_offset_y_ex[i];
				params->cam_horizon_r[i] += state->options.cam_horizon_r_ex[i];
			}
			params->cam_horizon_r[i] *= state->camera_horizon_r_bias;
			params->cam_aov[i] /= state->refraction;
		}
	}
}

//TODO
//static void add_renderer(RENDERER_T *renderer) {
//	int common_cur = 0;
//	char common[256];
//#ifdef USE_GLES
//	common_cur += sprintf(common + common_cur, "#version 100\n");
//	if(state->options.is_samplerExternalOES) {
//		common_cur += sprintf(common + common_cur, "#extension GL_OES_EGL_image_external: require\n");
//		common_cur += sprintf(common + common_cur, "#define cam_sampler2D samplerExternalOES\n");
//	} else {
//		common_cur += sprintf(common + common_cur, "#define cam_sampler2D sampler2D\n");
//	}
//#else
//	common_cur += sprintf(common + common_cur, "#version 330\n");
//	common_cur += sprintf(common + common_cur, "#define cam_sampler2D sampler2D\n");
//#endif
//	renderer->init(renderer, common, state->num_of_cam);
//
//	for (int i = 0; state->renderers[i] != (void*) -1; i++) {
//		if (state->renderers[i] == NULL) {
//			state->renderers[i] = renderer;
//			if (state->renderers[i + 1] == (void*) -1) {
//				int space = (i + 2) * 2;
//				if (space > 256) {
//					fprintf(stderr, "error on add_renderer\n");
//					return;
//				}
//				RENDERER_T **current = state->renderers;
//				state->renderers = malloc(sizeof(RENDERER_T*) * space);
//				memset(state->renderers, 0, sizeof(RENDERER_T*) * space);
//				memcpy(state->renderers, current, sizeof(RENDERER_T*) * i);
//				state->renderers[space - 1] = (void*) -1;
//				free(current);
//			}
//			return;
//		}
//	}
//}

void init_egl(EGL_HANDLER_T *state) {
#ifdef USE_GLES
#ifdef BCM_HOST
	int32_t success = 0;
	EGLBoolean result;
	EGLint num_config;

	static EGL_DISPMANX_WINDOW_T nativewindow;

	DISPMANX_ELEMENT_HANDLE_T dispman_element;
	DISPMANX_DISPLAY_HANDLE_T dispman_display;
	DISPMANX_UPDATE_HANDLE_T dispman_update;
	VC_RECT_T dst_rect;
	VC_RECT_T src_rect;

	static const EGLint attribute_list[] = {EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8, EGL_DEPTH_SIZE, 16,
		//EGL_SAMPLES, 4,
		EGL_SURFACE_TYPE, EGL_WINDOW_BIT, EGL_NONE};
	static const EGLint context_attributes[] = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE};

	EGLConfig config;

	// get an EGL display connection
	state->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
	assert(state->display != EGL_NO_DISPLAY);

	// initialize the EGL display connection
	result = eglInitialize(state->display, NULL, NULL);
	assert(EGL_FALSE != result);

	// get an appropriate EGL frame buffer configuration
	// this uses a BRCM extension that gets the closest match, rather than standard which returns anything that matches
	result = eglSaneChooseConfigBRCM(state->display, attribute_list, &config, 1, &num_config);
	assert(EGL_FALSE != result);

	//Bind to the right EGL API.
	result = eglBindAPI(EGL_OPENGL_ES_API);
	assert(result != EGL_FALSE);

	// create an EGL rendering context
	state->context = eglCreateContext(state->display, config, EGL_NO_CONTEXT, context_attributes);
	assert(state->context != EGL_NO_CONTEXT);

	// create an EGL window surface
	success = graphics_get_display_size(0 /* LCD */, &state->screen_width, &state->screen_height);
	assert(success >= 0);

	dst_rect.x = 0;
	dst_rect.y = 0;
	dst_rect.width = state->screen_width;
	dst_rect.height = state->screen_height;

	src_rect.x = 0;
	src_rect.y = 0;
	src_rect.width = state->screen_width << 16;
	src_rect.height = state->screen_height << 16;

	//if (state->preview)
	{
		dispman_display = vc_dispmanx_display_open(0 /* LCD */);
		dispman_update = vc_dispmanx_update_start(0);

		dispman_element = vc_dispmanx_element_add(dispman_update, dispman_display, 0/*layer*/, &dst_rect, 0/*src*/, &src_rect, DISPMANX_PROTECTION_NONE, 0 /*alpha*/, 0/*clamp*/, 0/*transform*/);

		nativewindow.element = dispman_element;
		nativewindow.width = state->screen_width;
		nativewindow.height = state->screen_height;
		vc_dispmanx_update_submit_sync(dispman_update);

		state->surface = eglCreateWindowSurface(state->display, config, &nativewindow, NULL);
	}
//	else {
//		//Create an offscreen rendering surface
//		EGLint rendering_attributes[] = { EGL_WIDTH, state->screen_width,
//				EGL_HEIGHT, state->screen_height, EGL_NONE };
//		state->surface = eglCreatePbufferSurface(state->display, config,
//				rendering_attributes);
//	}
	assert(state->surface != EGL_NO_SURFACE);
#elif TEGRA
	EGLBoolean result;
	EGLint num_config;
	static const EGLint attribute_list[] = {
		EGL_RED_SIZE, 8, //
		EGL_GREEN_SIZE, 8,//
		EGL_BLUE_SIZE, 8,//
		EGL_ALPHA_SIZE, 8,//
		EGL_DEPTH_SIZE, 16,//
		//EGL_SAMPLES, 4,
		EGL_SURFACE_TYPE, EGL_WINDOW_BIT,//
		EGL_NONE};
	static const EGLint context_attributes[] = {
		EGL_CONTEXT_CLIENT_VERSION, 2, //
		EGL_NONE};

	// get an EGL display connection
	state->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
	assert(state->display != EGL_NO_DISPLAY);

	// initialize the EGL display connection
	result = eglInitialize(state->display, NULL, NULL);
	assert(EGL_FALSE != result);

	// get an appropriate EGL frame buffer configuration
	result = eglChooseConfig(state->display, attribute_list, &state->config, 1, &num_config);
	assert(EGL_FALSE != result);

	//Bind to the right EGL API.
	result = eglBindAPI(EGL_OPENGL_ES_API);
	assert(result != EGL_FALSE);

	// create an EGL rendering context
	state->context = eglCreateContext(state->display, state->config, EGL_NO_CONTEXT, context_attributes);
	assert(state->context != EGL_NO_CONTEXT);

	//Create an offscreen rendering surface
	EGLint rendering_attributes[] = {EGL_WIDTH, state->screen_width,
		EGL_HEIGHT, state->screen_height, EGL_NONE};
	state->surface = eglCreatePbufferSurface(state->display, state->config,
			rendering_attributes);

	state->screen_width = 640;
	state->screen_height = 480;
#endif
#ifndef CONTEXT_SHARING
	// connect the context to the surface
	result = eglMakeCurrent(state->display, state->surface, state->surface, state->context);
	assert(EGL_FALSE != result);
	result = eglMakeCurrent(state->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
	assert(EGL_FALSE != result);
	result = eglMakeCurrent(state->display, state->surface, state->surface, state->context);
	assert(EGL_FALSE != result);
#endif
	state->context_tid = pthread_self();
	state->plugin_host.lock_texture();
	{
		// Enable back face culling.
		glEnable(GL_CULL_FACE);
	}
	state->plugin_host.unlock_texture();
#else
	glfwSetErrorCallback(glfwErrorCallback);
	if (glfwInit() == GL_FALSE) {
		printf("error on glfwInit\n");
	}
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	state->screen_width = 640;
	state->screen_height = 480;
	state->glfw_window = glfwCreateWindow(state->screen_width, state->screen_height, "picam360", NULL, NULL);

	glfwMakeContextCurrent(state->glfw_window);

	glewExperimental = GL_TRUE; //avoid glGenVertexArrays crash with glew-1.13
	if (glewInit() != GLEW_OK) {
		printf("error on glewInit\n");
	}
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
#endif
}

/***********************************************************
 * Name: init_model_proj
 *
 * Arguments:
 *       PICAM360CAPTURE_T *state - holds OGLES model info
 *
 * Description: Sets the OpenGL|ES model to default values
 *
 * Returns: void
 *
 ***********************************************************/
//TODO
//extern void create_calibration_renderer(PLUGIN_HOST_T *plugin_host, STREAMER_T **out_renderer);
//extern void create_board_renderer(PLUGIN_HOST_T *plugin_host, STREAMER_T **out_renderer);
static void init_model_proj(PICAM360CAPTURE_T *state) {
//	{
//		STREAMER_T *renderer = NULL;
//		create_calibration_renderer(&state->plugin_host, &renderer);
//		state->plugin_host.add_streamer(renderer);
//	}
//	{
//		STREAMER_T *renderer = NULL;
//		create_board_renderer(&state->plugin_host, &renderer);
//		state->plugin_host.add_streamer(renderer);
//	}
}

/***********************************************************
 * Name: init_textures
 *
 * Arguments:
 *       PICAM360CAPTURE_T *state - holds OGLES model info
 *
 * Description:   Initialise OGL|ES texture surfaces to use image
 *                buffers
 *
 * Returns: void
 *
 ***********************************************************/
static void deinit_textures(PICAM360CAPTURE_T *state) {
	for (int i = 0; i < state->num_of_cam; i++) {
		for (int j = 0; j < TEXTURE_BUFFER_NUM; j++) {
			if (state->cam_texture[i][j] != 0) {
				glBindTexture(GL_TEXTURE_2D, 0);
				glDeleteTextures(1, &state->cam_texture[i][j]);
				state->cam_texture[i][j] = 0;
			}
		}
	}
}
static void init_textures(PICAM360CAPTURE_T *state) {

	{
		const char *tmp_filepath = "/tmp/tmp.png";
		int *fd = open(tmp_filepath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IXOTH);
		write(fd, logo_png, logo_png_len);
		close(fd);
		load_texture(tmp_filepath, &state->logo_texture);

		memset(&state->logo_image, 0, sizeof(PICAM360_IMAGE_T));
		state->logo_image.img_type = PICAM360_IMAGE_TYPE_RGBA;
		state->logo_image.num_of_planes = 1;
		load_png(tmp_filepath, &state->logo_image.pixels[0], &state->logo_image.width[0], &state->logo_image.height[0], &state->logo_image.stride[0]);

		remove(tmp_filepath);
	}

	for (int i = 0; i < state->num_of_cam; i++) {
		for (int j = 0; j < TEXTURE_BUFFER_NUM; j++) {
			glGenTextures(1, &state->cam_texture[i][j]);

			glBindTexture(GL_TEXTURE_2D, state->cam_texture[i][j]);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, state->cam_width, state->cam_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		}
	}
}

void frame_handler() {
	struct timeval s, f;
	double elapsed_ms;
	bool snap_finished = false;
	FRAME_T **frame_pp = &state->frame;
	while (*frame_pp) {
		FRAME_INFO_T frame_info;
		FRAME_T *frame = *frame_pp;
		gettimeofday(&s, NULL);

		if (frame->fps > 0) {
			struct timeval diff;
			timersub(&s, &frame->last_updated, &diff);
			float diff_sec = (float) diff.tv_sec + (float) diff.tv_usec / 1000000;
			if (diff_sec < 1.0 / frame->fps) {
				frame_pp = &frame->next;
				continue;
			}
		}

		//start & stop recording
		if (frame->is_recording && frame->output_mode == OUTPUT_MODE_NONE) { //stop record

			if (frame->encoder) {
				frame->encoder->release(frame->encoder);
				frame->encoder = NULL;
			}

			frame->frame_elapsed /= frame->frame_num;
			printf("stop record : frame num : %d : fps %.3lf\n", frame->frame_num, 1000.0 / frame->frame_elapsed);

			frame->output_mode = OUTPUT_MODE_NONE;
			frame->is_recording = false;
			frame->delete_after_processed = true;
			if (frame->output_fd >= 0) {
				close(frame->output_fd);
				frame->output_fd = -1;
			}
		}
		if (!frame->is_recording && frame->output_mode == OUTPUT_MODE_VIDEO) {
			int ratio = frame->double_size ? 2 : 1;
			float fps = MAX(frame->fps, 1);
			if (frame->encoder) {
				frame->encoder->init(frame->encoder, frame->width * ratio, frame->height, 4000 * ratio, fps, stream_callback, NULL);
			}
			frame->frame_num = 0;
			frame->frame_elapsed = 0;
			frame->is_recording = true;
			frame->output_start = false;
			frame->output_fd = open(frame->output_filepath, O_CREAT | O_WRONLY | O_TRUNC, /*  */
			S_IRUSR | S_IWUSR | /* rw */
			S_IRGRP | S_IWGRP | /* rw */
			S_IROTH | S_IXOTH);
			printf("start_record saved to %s\n", frame->output_filepath);
		}
		if (!frame->is_recording && frame->output_mode == OUTPUT_MODE_STREAM) {
			int ratio = frame->double_size ? 2 : 1;
			float fps = MAX(frame->fps, 1);
			float kbps = frame->kbps;
			if (kbps == 0) {
				float ave_sq = sqrt((float) frame->width * (float) frame->height) / 1.2;
				if (frame->output_type == OUTPUT_TYPE_H265) {
					if (ave_sq <= 240) {
						kbps = 100;
					} else if (ave_sq <= 320) {
						kbps = 200;
					} else if (ave_sq <= 480) {
						kbps = 400;
					} else if (ave_sq <= 640) {
						kbps = 800;
					} else if (ave_sq <= 960) {
						kbps = 1600;
					} else {
						kbps = 3200;
					}
				} else if (frame->output_type == OUTPUT_TYPE_H264) {
					if (ave_sq <= 240) {
						kbps = 200;
					} else if (ave_sq <= 320) {
						kbps = 400;
					} else if (ave_sq <= 480) {
						kbps = 800;
					} else if (ave_sq <= 640) {
						kbps = 1600;
					} else if (ave_sq <= 960) {
						kbps = 3200;
					} else {
						kbps = 6400;
					}
				} else {
					if (ave_sq <= 240) {
						kbps = 800;
					} else if (ave_sq <= 320) {
						kbps = 1600;
					} else if (ave_sq <= 480) {
						kbps = 3200;
					} else if (ave_sq <= 640) {
						kbps = 6400;
					} else if (ave_sq <= 960) {
						kbps = 12800;
					} else {
						kbps = 25600;
					}
				}
			}
			if (frame->encoder) {
				frame->encoder->init(frame->encoder, frame->width * ratio, frame->height, kbps * ratio, fps, stream_callback, frame);
			}
			frame->frame_num = 0;
			frame->frame_elapsed = 0;
			frame->is_recording = true;
			printf("start_record saved to %s : %d kbps\n", frame->output_filepath, (int) kbps);
		}

		if (frame->renderer->render2encoder == NULL) { //rendering to buffer
			VECTOR4D_T view_quat = { .ary = { 0, 0, 0, 1 } };
			if (frame->view_mpu) {
				view_quat = frame->view_mpu->get_quaternion(frame->view_mpu);
			}

			{ //store info
				memcpy(frame_info.client_key, frame->client_key, sizeof(frame_info.client_key));
				frame_info.server_key = frame->server_key;
				gettimeofday(&frame_info.before_redraw_render_texture, NULL);
				frame_info.fov = frame->fov;
				frame_info.view_quat = view_quat;
			}

			state->plugin_host.lock_texture();
			if (frame->double_size) {
				int size = frame->width * frame->height * 3;
				unsigned char *image_buffer = (unsigned char*) malloc(size);
				unsigned char *image_buffer_double = frame->img_buff;
				frame->img_width = frame->width * 2;
				frame->img_height = frame->height;
				for (int split = 0; split < 2; split++) {
					state->split = split + 1;

					glBindFramebuffer(GL_FRAMEBUFFER, frame->framebuffer);
					redraw_render_texture(state, frame, frame->renderer, view_quat);
					glFinish();
					glReadPixels(0, 0, frame->width, frame->height, GL_RGB, GL_UNSIGNED_BYTE, image_buffer);
					glBindFramebuffer(GL_FRAMEBUFFER, 0);
					for (int y = 0; y < frame->height; y++) {
						memcpy(image_buffer_double + frame->width * 2 * 3 * y + frame->width * 3 * split, image_buffer + frame->width * 3 * y, frame->width * 3);
					}
				}
				free(image_buffer);
			} else {
				unsigned char *image_buffer = frame->img_buff;
				frame->img_width = frame->width;
				frame->img_height = frame->height;
				state->split = 0;

				{
					glBindFramebuffer(GL_FRAMEBUFFER, frame->framebuffer);
					redraw_render_texture(state, frame, frame->renderer, view_quat);
					if (state->menu_visible) {
						redraw_info(state, frame);
					}
					glFinish();
					glReadPixels(0, 0, frame->width, frame->height, GL_RGB, GL_UNSIGNED_BYTE, image_buffer);
					glBindFramebuffer(GL_FRAMEBUFFER, 0);
				}
			}
			state->plugin_host.unlock_texture();

			{ //store info
				gettimeofday(&frame_info.after_redraw_render_texture, NULL);
			}
		}

		switch (frame->output_mode) {
		case OUTPUT_MODE_STILL:
			break;
		case OUTPUT_MODE_VIDEO:
			break;
		case OUTPUT_MODE_STREAM:
			break;
		default:
			break;
		}
		if (frame->after_processed_callback) {
			frame->after_processed_callback(state, frame);
		}
		//next rendering
		if (frame->delete_after_processed) {
			*frame_pp = frame->next;
			delete_frame(frame);
			frame = NULL;
		} else {
			frame_pp = &frame->next;
		}
		//preview
		if (frame && frame == state->frame && state->preview) {
			state->plugin_host.lock_texture();
			{
				redraw_scene(state, frame, &state->model_data[OPERATION_MODE_BOARD]);
			}
			state->plugin_host.unlock_texture();
#ifdef USE_GLES
			eglSwapBuffers(state->display, state->surface);
#else
			glfwSwapBuffers(state->glfw_window);
			glfwPollEvents();
#endif
		}
		if (frame) {
			frame->last_updated = s;
		}
	}
	if (snap_finished) {
		state->plugin_host.send_event(PICAM360_HOST_NODE_ID, PICAM360_CAPTURE_EVENT_AFTER_SNAP);
	}
	state->plugin_host.send_event(PICAM360_HOST_NODE_ID, PICAM360_CAPTURE_EVENT_AFTER_FRAME);
}

static void redraw_info(PICAM360CAPTURE_T *state, FRAME_T *frame) {
	int frame_width = (frame->stereo) ? frame->width / 2 : frame->width;
	int frame_height = frame->height;
	const int MAX_INFO_SIZE = 1024;
	char disp[MAX_INFO_SIZE];
	get_info_str(disp, MAX_INFO_SIZE);
	menu_redraw(state->menu, disp, frame_width, frame_height, frame_width, frame_height, false);
}
/***********************************************************
 * Name: redraw_scene
 *
 * Arguments:
 *       PICAM360CAPTURE_T *state - holds OGLES model info
 *
 * Description:   Draws the model and calls eglSwapBuffers
 *                to render to screen
 *
 * Returns: void
 *
 ***********************************************************/
static void redraw_render_texture(PICAM360CAPTURE_T *state, FRAME_T *frame, RENDERER_T *renderer, VECTOR4D_T view_quat) {
	if (renderer == NULL) {
		return;
	}
	int CAM_GL_TEXTURE_2D = (state->options.is_samplerExternalOES) ? GL_TEXTURE_EXTERNAL_OES : GL_TEXTURE_2D;

	int frame_width = (frame->stereo) ? frame->width / 2 : frame->width;
	int frame_height = frame->height;

	int program = renderer->get_program(renderer);
	glUseProgram(program);

	glViewport(0, 0, frame_width, frame_height);

	{ // bind texture
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, state->logo_texture);
		glUniform1i(glGetUniformLocation(program, "logo_texture"), 0);

		int cam_texture[MAX_CAM_NUM];
		for (int i = 0; i < state->num_of_cam; i++) {
			cam_texture[i] = i + 1;
			glActiveTexture(GL_TEXTURE1 + i);
			glBindTexture(CAM_GL_TEXTURE_2D, state->cam_texture[i][state->cam_texture_cur[i]]);
		}
		glUniform1iv(glGetUniformLocation(program, "cam_texture"), state->num_of_cam, cam_texture);
	}

	{ //cam_attitude //depth axis is z, vertical asis is y
		float cam_attitude[16 * MAX_CAM_NUM];
		float view_matrix[16];
		float north_matrix[16];
		float world_matrix[16];

		{ // Rv : view
			mat4_identity(view_matrix);
			mat4_fromQuat(view_matrix, view_quat.ary);
			mat4_invert(view_matrix, view_matrix);
		}

		{ // Rn : north
			mat4_identity(north_matrix);
			float north = state->plugin_host.get_view_north();
			mat4_rotateY(north_matrix, north_matrix, north * M_PI / 180);
		}

		{ // Rw : view coodinate to world coodinate and view heading to ground initially
			mat4_identity(world_matrix);
			mat4_rotateX(world_matrix, world_matrix, -M_PI / 2);
		}

		for (int i = 0; i < state->num_of_cam; i++) {
			float *unif_matrix = cam_attitude + 16 * i;
			float cam_matrix[16];
			{ // Rc : cam orientation
				mat4_identity(cam_matrix);
				if (state->camera_coordinate_from_device) {
					mat4_fromQuat(cam_matrix, state->camera_quaternion[i].ary);
				} else {
					//euler Y(yaw)X(pitch)Z(roll)
					mat4_rotateZ(cam_matrix, cam_matrix, state->camera_roll);
					mat4_rotateX(cam_matrix, cam_matrix, state->camera_pitch);
					mat4_rotateY(cam_matrix, cam_matrix, state->camera_yaw);
				}
			}

			{ // Rco : cam offset  //euler Y(yaw)X(pitch)Z(roll)
				float cam_offset_matrix[16];
				mat4_identity(cam_offset_matrix);
				mat4_rotateZ(cam_offset_matrix, cam_offset_matrix, state->options.cam_offset_roll[i]);
				mat4_rotateX(cam_offset_matrix, cam_offset_matrix, state->options.cam_offset_pitch[i]);
				mat4_rotateY(cam_offset_matrix, cam_offset_matrix, state->options.cam_offset_yaw[i]);
				mat4_invert(cam_offset_matrix, cam_offset_matrix);
				mat4_multiply(cam_matrix, cam_matrix, cam_offset_matrix); // Rc'=RcoRc
			}

			{ //RcRv(Rc^-1)RcRw
				mat4_identity(unif_matrix);
				mat4_multiply(unif_matrix, unif_matrix, world_matrix); // Rw
				mat4_multiply(unif_matrix, unif_matrix, view_matrix); // RvRw
				//mat4_multiply(unif_matrix, unif_matrix, north_matrix); // RnRvRw
				mat4_multiply(unif_matrix, unif_matrix, cam_matrix); // RcRnRvRw
			}
			mat4_transpose(unif_matrix, unif_matrix); // this mat4 library is row primary, opengl is column primary
		}
		glUniformMatrix4fv(glGetUniformLocation(program, "cam_attitude"), state->num_of_cam, GL_FALSE, (GLfloat*) cam_attitude);
	}
	{ //cam_options
		float cam_offset_yaw[MAX_CAM_NUM];
		float cam_offset_x[MAX_CAM_NUM];
		float cam_offset_y[MAX_CAM_NUM];
		float cam_horizon_r[MAX_CAM_NUM];
		float cam_aov[MAX_CAM_NUM];
		for (int i = 0; i < state->num_of_cam; i++) {
			cam_offset_x[i] = state->options.cam_offset_x[i];
			cam_offset_y[i] = state->options.cam_offset_y[i];
			cam_horizon_r[i] = state->options.cam_horizon_r[i];
			cam_aov[i] = state->options.cam_aov[i];
			if (state->options.config_ex_enabled) {
				cam_offset_x[i] += state->options.cam_offset_x_ex[i];
				cam_offset_y[i] += state->options.cam_offset_y_ex[i];
				cam_horizon_r[i] += state->options.cam_horizon_r_ex[i];
			}
			cam_horizon_r[i] *= state->camera_horizon_r_bias;
			cam_aov[i] /= state->refraction;
		}
		glUniform1fv(glGetUniformLocation(program, "cam_offset_x"), state->num_of_cam, cam_offset_x);
		glUniform1fv(glGetUniformLocation(program, "cam_offset_y"), state->num_of_cam, cam_offset_y);
		glUniform1fv(glGetUniformLocation(program, "cam_horizon_r"), state->num_of_cam, cam_horizon_r);
		glUniform1fv(glGetUniformLocation(program, "cam_aov"), state->num_of_cam, cam_aov);

		glUniform1i(glGetUniformLocation(program, "active_cam"), state->active_cam);
		glUniform1i(glGetUniformLocation(program, "num_of_cam"), state->num_of_cam);
	}

	//these should be into each plugin
	//Load in the texture and thresholding parameters.
	glUniform1f(glGetUniformLocation(program, "split"), state->split);
	glUniform1f(glGetUniformLocation(program, "pixel_size"), 1.0 / state->cam_width);

	glUniform1f(glGetUniformLocation(program, "cam_aspect_ratio"), (float) state->cam_width / (float) state->cam_height);
	glUniform1f(glGetUniformLocation(program, "frame_aspect_ratio"), (float) frame_width / (float) frame_height);

	glUniform1f(glGetUniformLocation(program, "sharpness_gain"), state->options.sharpness_gain);
	glUniform1f(glGetUniformLocation(program, "color_offset"), state->options.color_offset);
	glUniform1f(glGetUniformLocation(program, "color_factor"), 1.0 / (1.0 - state->options.color_offset));
	glUniform1f(glGetUniformLocation(program, "overlap"), state->options.overlap);

	glDisable(GL_BLEND);
	glEnable(GL_CULL_FACE);

	renderer->render(renderer, frame->fov);

	glFlush();
}

static void redraw_scene(PICAM360CAPTURE_T *state, FRAME_T *frame, MODEL_T *model) {
	int frame_width = (frame->stereo) ? frame->width / 2 : frame->width;
	int frame_height = frame->height;

	int program = GLProgram_GetId(model->program);
	glUseProgram(program);

	// Start with a clear screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glBindBuffer(GL_ARRAY_BUFFER, model->vbo);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, frame->texture);

	//Load in the texture and thresholding parameters.
	glUniform1i(glGetUniformLocation(program, "tex"), 0);
	glUniform1f(glGetUniformLocation(program, "tex_scalex"), (frame->stereo) ? 0.5 : 1.0);

#ifdef USE_GLES
	GLuint loc = glGetAttribLocation(program, "vPosition");
	glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(loc);
#else
	glBindVertexArray(model->vao);
#endif

	glDisable(GL_BLEND);
	glEnable(GL_CULL_FACE);

	if (strcasecmp(frame->renderer->name, "CALIBRATION") == 0) {
		glViewport((state->screen_width - state->screen_height) / 2, 0, (GLsizei) state->screen_height, (GLsizei) state->screen_height);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, model->vbo_nop);
	} else if (frame->stereo) {
		int offset_x = (state->screen_width / 2 - frame_width) / 2;
		int offset_y = (state->screen_height - frame_height) / 2;
		for (int i = 0; i < 2; i++) {
			//glViewport(0, 0, (GLsizei)state->screen_width/2, (GLsizei)state->screen_height);
			glViewport(offset_x + i * state->screen_width / 2, offset_y, (GLsizei) frame_width, (GLsizei) frame_height);
			glDrawArrays(GL_TRIANGLE_STRIP, 0, model->vbo_nop);
		}
	} else {
		int offset_x = (state->screen_width - frame_width) / 2;
		int offset_y = (state->screen_height - frame_height) / 2;
		glViewport(offset_x, offset_y, (GLsizei) frame_width, (GLsizei) frame_height);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, model->vbo_nop);
	}

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindTexture(GL_TEXTURE_2D, 0);
#ifdef USE_GLES
	glDisableVertexAttribArray(loc);
#else
	glBindVertexArray(0);
#endif
}
