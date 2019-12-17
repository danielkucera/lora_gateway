/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>		/* C99 types */
#include <stdio.h>		/* printf fprintf */
#include <stdlib.h>		/* malloc free */
#include <string.h>		/* memcpy */

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "loragw_spi.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_SPI == 1
	#define DEBUG_MSG(str)				fprintf(stderr, str)
	#define DEBUG_PRINTF(fmt, args...)	fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
	#define CHECK_NULL(a)				if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_SPI_ERROR;}
#else
	#define DEBUG_MSG(str)
	#define DEBUG_PRINTF(fmt, args...)
	#define CHECK_NULL(a)				if(a==NULL){return LGW_SPI_ERROR;}
#endif

#define READ_CMD	0x01
#define WRITE_CMD	0x02
#define BURST_WRITE_CMD	0x03
#define BURST_READ_CMD	0x04

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define READ_ACCESS		0x00
#define WRITE_ACCESS	0x80

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int spi_sock = -1;

/* SPI initialization and configuration */
int lgw_spi_open(void **spi_target_ptr) {
	struct sockaddr_un server;

	DEBUG_MSG("lgw_spi_open\n");
	/* check input variables */
	CHECK_NULL(spi_target_ptr); /* cannot be null, must point on a void pointer (*spi_target_ptr can be null) */
	
	spi_sock = socket(AF_UNIX, SOCK_STREAM, 0);
	if (spi_sock < 0) {
		DEBUG_MSG("ERROR opening stream socket\n");
		return LGW_SPI_ERROR;
	}
	server.sun_family = AF_UNIX;
	strcpy(server.sun_path, "/var/run/lora.sock");

	if (connect(spi_sock, (struct sockaddr *) &server, sizeof(struct sockaddr_un)) < 0) {
		close(spi_sock);
		DEBUG_MSG("ERROR connecting stream socket");
		return LGW_SPI_ERROR;
	}

	*spi_target_ptr = (void *)&spi_sock; // we cannot left this ptr empty

	return LGW_SPI_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* SPI release */
int lgw_spi_close(void *spi_target) {
	DEBUG_MSG("lgw_spi_close\n");
	
	close(spi_sock);
	
	/* close return no status, assume success (0_o) */
	return LGW_SPI_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int handle_cmd(uint8_t *out_buf, uint8_t *in_buf, int len){
	DEBUG_PRINTF("Info: handling SPI on socket %d\n", spi_sock);
	int ret;

	if (write(spi_sock, out_buf, len) < len){
		DEBUG_MSG("ERROR: SPI CMD SEND FAILURE\n");
		return LGW_SPI_ERROR;
	}
	if ((ret = read(spi_sock, in_buf, len)) != len){
		DEBUG_PRINTF("ERROR: SPI CMD short response %d expected %d\n", ret, len);
		return LGW_SPI_ERROR;
	}
	if (in_buf[0] != 0xff){
		DEBUG_PRINTF("ERROR: SPI CMD failed with errno %x\n", in_buf[0]);
		return LGW_SPI_ERROR;
	}

	DEBUG_MSG("Note: SPI cmd success\n");
	return LGW_SPI_SUCCESS;
}

/* Simple write */
int lgw_spi_w(void *spi_target, uint8_t spi_mux_mode, uint8_t spi_mux_target, uint8_t address, uint8_t data) {
	DEBUG_MSG("lgw_spi_w\n");
	uint8_t in_buf[6];
	uint8_t out_buf[6];
	uint8_t command_size;
	
	/* check input variables */
	if ((address & 0x80) != 0) {
		DEBUG_MSG("WARNING: SPI address > 127\n");
	}
	
    /* prepare frame to be sent */
    if (spi_mux_mode == LGW_SPI_MUX_MODE1) {
        out_buf[3] = spi_mux_target;
        out_buf[4] = WRITE_ACCESS | (address & 0x7F);
        out_buf[5] = data;
        command_size = 6;
    } else {
        out_buf[3] = WRITE_ACCESS | (address & 0x7F);
        out_buf[4] = data;
        command_size = 5;
    }
	
	out_buf[0] = WRITE_CMD;
	out_buf[1] = 0;
	out_buf[2] = command_size & 0xff;

	return handle_cmd(out_buf, in_buf, command_size);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple read (using Transfer function) */
int lgw_spi_r(void *spi_target, uint8_t spi_mux_mode, uint8_t spi_mux_target, uint8_t address, uint8_t *data) {
	DEBUG_MSG("lgw_spi_r\n");
	uint8_t in_buf[6];
	uint8_t out_buf[6];
	uint8_t command_size;
	
	/* check input variables */
	if ((address & 0x80) != 0) {
		DEBUG_MSG("WARNING: SPI address > 127\n");
	}
	CHECK_NULL(data);
	
	/* prepare frame to be sent */
	if (spi_mux_mode == LGW_SPI_MUX_MODE1) {
		out_buf[3] = spi_mux_target;
		out_buf[4] = READ_ACCESS | (address & 0x7F);
		out_buf[5] = 0x00;
		command_size = 6;
	} else {
		out_buf[3] = READ_ACCESS | (address & 0x7F);
		out_buf[4] = 0x00;
		command_size = 5;
	}

	out_buf[0] = READ_CMD;
	out_buf[1] = 0;
	out_buf[2] = command_size & 0xff;

	int ret = handle_cmd(out_buf, in_buf, command_size);
	*data = in_buf[4];

	return ret;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) write */
int lgw_spi_wb(void *spi_target, uint8_t spi_mux_mode, uint8_t spi_mux_target, uint8_t address, uint8_t *data, uint16_t size) {
	DEBUG_MSG("lgw_spi_wb\n");
	DEBUG_PRINTF("total len %d\n", size);
	uint8_t command[6];
	uint8_t command_size;
	uint8_t *out_buf = NULL;
	int size_to_do, buf_size, chunk_size, offset;
	int i;
	int ret;
	
	/* check input parameters */
	if ((address & 0x80) != 0) {
		DEBUG_MSG("WARNING: SPI address > 127\n");
	}
	CHECK_NULL(data);
	if (size == 0) {
		DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
		return LGW_SPI_ERROR;
	}

    /* prepare command bytes */
    if (spi_mux_mode == LGW_SPI_MUX_MODE1) {
        command[3] = spi_mux_target;
        command[4] = WRITE_ACCESS | (address & 0x7F);
        command_size = 5;
    } else {
        command[3] = WRITE_ACCESS | (address & 0x7F);
        command_size = 4;
    }
	size_to_do = size + command_size; /* add a byte for the address */

	command[0] = BURST_WRITE_CMD;
	command[1] = (size_to_do >> 8) & 0xff;
	command[2] = size_to_do & 0xff;

	out_buf = malloc(size_to_do);
	if (out_buf == NULL) {
		DEBUG_MSG("ERROR: MALLOC FAIL\n");
		return LGW_SPI_ERROR;
	}

	uint8_t in_buf[size_to_do];

	memcpy(out_buf, command, command_size);
        memcpy(out_buf+command_size, data, size);
		
	ret = handle_cmd(out_buf, in_buf, size_to_do);
	if (ret != LGW_SPI_SUCCESS){
		return ret;
	}

	/* deallocate data buffer */
	free(out_buf);
	
	return ret;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) read */
int lgw_spi_rb(void *spi_target, uint8_t spi_mux_mode, uint8_t spi_mux_target, uint8_t address, uint8_t *data, uint16_t size) {
	DEBUG_MSG("lgw_spi_rb\n");
	DEBUG_PRINTF("total len %d\n", size);
	uint8_t command[4];
	uint8_t command_size;
	int size_to_do, chunk_size, offset;
	int i;
	int ret;
	
	/* check input parameters */
	if ((address & 0x80) != 0) {
		DEBUG_MSG("WARNING: SPI address > 127\n");
	}
	CHECK_NULL(data);
	if (size == 0) {
		DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
		return LGW_SPI_ERROR;
	}
	
	/* prepare command bytes */
    if (spi_mux_mode == LGW_SPI_MUX_MODE1) {
        command[3] = spi_mux_target;
        command[4] = READ_ACCESS | (address & 0x7F);
        command_size = 5;
    } else {
        command[3] = READ_ACCESS | (address & 0x7F);
        command_size = 4;
    }
	size_to_do = size + command_size;

	command[0] = BURST_READ_CMD;
	command[1] = (size_to_do >> 8) & 0xff;
	command[2] = size_to_do & 0xff;

	uint8_t out_buf[size_to_do];
	uint8_t in_buf[size_to_do];
        memcpy(out_buf, command, command_size);

	ret = handle_cmd(out_buf, in_buf, size_to_do);
	if (ret != LGW_SPI_SUCCESS){
		return ret;
	}

	memcpy(data, in_buf+command_size, size);

	return ret;
}

/* --- EOF ------------------------------------------------------------------ */
