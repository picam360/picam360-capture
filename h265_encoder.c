#include <pthread.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/prctl.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

typedef void (*RECORD_CALLBACK)(unsigned char *data, unsigned int data_len, void *frame_data, void *user_data);

typedef struct _h254_encoder {
	pid_t pid;
	int pin_fd;
	int pout_fd;
	int width;
	int height;
	pthread_t pout_thread;
	RECORD_CALLBACK callback;
	void *user_data;
} h254_encoder;

static void *pout_thread_func(void* arg) {
	unsigned int data_len = 0;
	unsigned int buff_size = 64 * 1024;
	unsigned char *data = malloc(buff_size);
	h254_encoder *encoder = (h254_encoder*) arg;
	while ((data_len = read(encoder->pout_fd, data, buff_size)) > 0) {
		if (encoder->callback) {
			encoder->callback(data, data_len, NULL, encoder->user_data);
		}
		//printf("%d:%s\n", data_len, data);
		if (data_len != buff_size) {
			printf("NAL %02x%02x%02x%02x\n", data[0], data[1], data[2], data[3]);
		} else {
			printf("not NAL %02x%02x%02x%02x\n", data[0], data[1], data[2], data[3]);
		}
		printf("%d\n", data_len);
	}
	return NULL;
}

#define R (0)
#define W (1)
h254_encoder *h265_create_encoder(const int width, const int height, int bitrate_kbps, int fps, RECORD_CALLBACK callback, void *user_data) {
	pid_t pid = 0;
	int pin_fd[2];
	int pout_fd[2];

	h254_encoder *encoder = malloc(sizeof(h254_encoder));
	encoder->width = width;
	encoder->height = height;

	pipe(pin_fd);
	pipe(pout_fd);
	pid = fork();
	if (pid == 0) {
		// Child
		dup2(pin_fd[R], STDIN_FILENO);
		dup2(pout_fd[W], STDOUT_FILENO);
		//dup2(pout_fd[W], STDERR_FILENO);

		//ask kernel to deliver SIGTERM in case the parent dies
		prctl(PR_SET_PDEATHSIG, SIGTERM);

		execlp("ffmpeg", "ffmpeg", "-f", "rawvideo", "-framerate", "5", "-video_size", "512x512", "-pix_fmt", "rgb24", "-i", "pipe:0", "-c:v", "libx265", "-pix_fmt", "yuv420p", "-preset", "ultrafast",
				"-tune", "zerolatency", "-vb", "256k", "-f", "rawvideo", "pipe:1", (char*) NULL);
		// Nothing below this line should be executed by child process. If so,
		// it means that the execl function wasn't successfull, so lets exit:
		exit(1);
	}
	// The code below will be executed only by parent. You can write and read
	// from the child using pipefd descriptors, and you can send signals to
	// the process using its pid by kill() function. If the child process will
	// exit unexpectedly, the parent process will obtain SIGCHLD signal that
	// can be handled (e.g. you can respawn the child process).

	//close unused pipe ends
	close(pin_fd[R]);
	close(pout_fd[W]);

	pthread_create(&encoder->pout_thread, NULL, pout_thread_func, (void*) encoder);

	encoder->pid = pid;
	encoder->pin_fd = pin_fd[W];
	encoder->pout_fd = pout_fd[R];
	return encoder;
}
void h265_delete_encoder(h254_encoder *encoder) {
	int status;
	kill(encoder->pid, SIGKILL); //send SIGKILL signal to the child process
	waitpid(encoder->pid, &status, 0);
}
void h265_add_frame(h254_encoder *encoder, const unsigned char *in_data) {
	write(encoder->pin_fd, in_data, encoder->width * encoder->height * 3);
}

int main(int argc, char *argv[]) {
	h254_encoder *encoder = h265_create_encoder(512, 512, 256, 5, NULL, NULL);
	int fd = open(argv[1], O_RDONLY);
	int size = 0;
	int frame_size = 512 * 512 * 3;
	unsigned char *frame_buffer = (unsigned char *) malloc(frame_size);
	while ((size = read(fd, frame_buffer, frame_size)) == frame_size) {
		h265_add_frame(encoder, frame_buffer);
		usleep(200 * 1000);
	}
	close(fd);
}
