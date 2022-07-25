/*
 * mcu_comunication.c
 *
 *  Created on: Aug 31, 2020
 *      Author: bobkov
 */

#include "mcu_communication.h"
#include <stdio.h>
#include <string.h>
#include "public.h"

static int find_cmd( unsigned char c, CAMCMD *cmd );
static int find_int( unsigned char c, unsigned char *buf, uint8_t *cnt, int *val );
static MCU_Status _process_char( CAM_COM *cam, unsigned char c );
static int _parse_set_laser( unsigned char c, CAMARG *arg );
static int _parse_time_led(unsigned char c, CAMARG *arg);


CMD_HANDL_CALLBACK handle_cmd = 0;
CAM_COM comm = {

		.arg = {0},
		.state = STATE_NONE,
		.foundCmd = CMD_NONE,
		.cmd = {
			{ .type = CMD_BEGIN, 			.cnt = 0, 			.cmd = "<TM>", 				.parse_arg = 0 },
			{ .type = CMD_END, 				.cnt = 0, 			.cmd = "<END>",				.parse_arg = 0 },
			{ .type = CMD_GET_STAT, 		.cnt = 0, 			.cmd = "GET_STAT", 			.parse_arg = 0 },
			{ .type = CMD_LAMP_ON, 			.cnt = 0, 		  	.cmd = "LAMP_ON", 			.parse_arg = 0 },
			{ .type = CMD_LAMP_OFF, 		.cnt = 0, 		  	.cmd = "LAMP_OFF", 			.parse_arg = 0 },
			{ .type = CMD_SET_LASER, 		.cnt = 0, 		 	.cmd = "SET_LASER:", 		.parse_arg = _parse_set_laser },
			{ .type = CMD_LED_WIFI_ON, 		.cnt = 0, 		  	.cmd = "LED_WIFI_ON", 		.parse_arg = 0 },
			{ .type = CMD_LED_WIFI_OFF, 	.cnt = 0, 		  	.cmd = "LED_WIFI_OFF", 		.parse_arg = 0 },
			{ .type = CMD_ZUMMER, 			.cnt = 0, 		  	.cmd = "ZUMMER:", 			.parse_arg = _parse_time_led },
			{ .type = CMD_COMPLETE, 		.cnt = 0,	 		.cmd = "COMPLETE", 			.parse_arg = 0 },
			{ .type = CMD_L_RED, 			.cnt = 0, 		  	.cmd = "L_RED:", 			.parse_arg = _parse_time_led },
			{ .type = CMD_L_GREEN, 			.cnt = 0, 		  	.cmd = "L_GREEN:", 			.parse_arg = _parse_time_led },
			{ .type = CMD_VIBRO, 			.cnt = 0, 		  	.cmd = "VIBRO:", 			.parse_arg = _parse_time_led },
			{ .type = CMD_SHUTDOWN, 		.cnt = 0,	 		.cmd = "SHUTDOWN", 			.parse_arg = 0 }

		}
};

static int _parse_set_laser( unsigned char c, CAMARG *arg ){

	static int val;
	int res= find_int(c,arg->buf,&arg->cnt,&val);
	if (res>0){
		arg->laser.Laser[0]=val;
		arg->argCnt++;
	}
	else if(res<0){
		return -1;
	}
	if (arg->argCnt>=1){
		return 1;
	}
	return 0;
}

static int _parse_time_led(unsigned char c, CAMARG *arg){
	static int val;
	int res = find_int(c,arg->buf,&arg->cnt,&val);
	if (res>0){
			arg->laser.Laser[0]=val;
			arg->argCnt++;
		}
		else if(res<0){
			return -1;
		}
		if (arg->argCnt>=1){
			return 1;
		}
		return 0;
}


MCU_Status MCU_HandleByte(unsigned char data){
	return _process_char( &comm, data );
}

static int find_cmd( unsigned char c, CAMCMD *cmd ){
	if ( cmd->cmd[cmd->cnt] == c ){
		cmd->cnt++;
	}else{
		if ( cmd->cnt != 0 ){
			cmd->cnt = 0;
			find_cmd( c, cmd );
		}
		return 0;
	}

	if ( (cmd->cnt) >= strlen(cmd->cmd) ){
		cmd->cnt = 0;
		return 1;
	}

	return 0;
}

static int find_int( unsigned char c, unsigned char *buf, uint8_t *cnt, int *val ){
	if ( *cnt >= MAX_STRING_BUF ){
		*cnt = 0;
	}

	if( (( c >= '0' ) && ( c <= '9' )) || ( c == '-' ) ){
		buf[*cnt] = c;
		*cnt += 1;
	}else if ((c == ',') || (c == 0) || (c == ' ') || ( c == '<')){
		uint8_t res = 0;

		if ( sscanf(buf,"%d",val) > 0 ){
			res = 1;
		}
		memset( buf, 0, MAX_STRING_BUF );
		*cnt = 0;
		return res;
	}else{
		return -1;
	}

	return 0;
}

static MCU_Status _process_char( CAM_COM *cam, unsigned char c ){
	if ( find_cmd( c, &cam->cmd[CMD_END] ) ){
		if ( ( cam->state != STATE_WAIT_EXEC ) || (handle_cmd == 0) ){
			cam->state = STATE_NONE;
			cam->foundCmd = CMD_NONE;
		}
	}

	if ( find_cmd( c, &cam->cmd[CMD_BEGIN] ) ){
		cam->state = STATE_CMD_DECODE_CMD;
		cam->foundCmd = CMD_NONE;
		cam->arg.argCnt = 0;
		cam->arg.cnt = 0;
		memset(cam->arg.buf, 0, MAX_STRING_BUF);
	}

	switch (cam->state){
		case STATE_NONE:{

		}break;

		case STATE_CMD_DECODE_CMD:{

			for ( int i = 2; i < ARRAY_SIZE(cam->cmd); i++ ){
				if ( find_cmd( c, &cam->cmd[i] ) ){
					cam->state = STATE_GET_ARGS;
					cam->foundCmd = i;
				}
			}

		}break;

		case STATE_GET_ARGS:{
			if ( cam->cmd[cam->foundCmd].parse_arg != 0 ){
				int res = cam->cmd[cam->foundCmd].parse_arg( c, &cam->arg );
				if ( res > 0 ){
					cam->state = STATE_WAIT_EXEC;
				}else if ( res < 0 ){
					cam->state = STATE_NONE;
				}
			}else{

				cam->state = STATE_WAIT_EXEC;
			}
		}break;

		case STATE_WAIT_EXEC:{
			//При выполнении колбека программа должна сбросить foundCmd, что является сигналом о том, что обработка команды закончена
			if ( cam->foundCmd == CMD_NONE ){
				cam->state = STATE_NONE;
			}
		}break;

		default:{
			cam->state = STATE_NONE;
			cam->foundCmd = CMD_NONE;
		}
	}

	return MCUSTAT_OK;
}

MCU_Status CAMCMD_Init( CMD_HANDL_CALLBACK callback ){
	if ( callback == 0 ){
		return MCUSTAT_ERR;
	}
	handle_cmd = callback;
	return MCUSTAT_OK;
}

void CAMCMD_ProcessMessages( void ){
		if ( comm.state == STATE_WAIT_EXEC && handle_cmd && comm.foundCmd != CMD_NONE ){
			handle_cmd( comm.foundCmd, &comm.arg );
			comm.foundCmd = CMD_NONE;
			memset(&comm.arg, 0, sizeof(CAMARG));
		}
}

