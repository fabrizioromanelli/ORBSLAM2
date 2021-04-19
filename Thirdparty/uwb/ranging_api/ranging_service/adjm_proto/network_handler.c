#include "network_handler.h"

volatile int sync_sent_status;
pthread_mutex_t snt_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t snt_cond = PTHREAD_COND_INITIALIZER;

static void sent(node_id_t dst, uint16_t handle, enum tx_status status) 
{
	// unused parameters. Only one sync_send at time can be executed
	(void) handle;
	(void) dst;

	if(pthread_mutex_lock( &snt_mutex ) < 0) {
		sync_sent_status = LOCK_ERROR;
		return;
	}
	if(status != TX_FAIL) { 
	// even with ack not received we use this function only for
	// broadcast so no ack is expected
		sync_sent_status = SYNC_SUCCESS;
	}
	else{
		sync_sent_status = SEND_FAILED;
	}
	if(pthread_cond_signal(&snt_cond) < 0) {
		sync_sent_status = LOCK_ERROR;
		return;
	}
	if(pthread_mutex_unlock(&snt_mutex) < 0) {
		sync_sent_status = LOCK_ERROR;
		return;
	}
}

void init_callbacks() 
{
	register_sent_cb(sent);
}

int sync_send(node_id_t target, uint8_t * buffer, int len)
{
	if(pthread_mutex_lock( &snt_mutex )  < 0) {
		return ERROR_SYSTEM;
	}
	enum out_queue_status s = send(target, buffer, len, 15);
	if(s != OQ_SUCCESS){
		if(pthread_mutex_unlock(&snt_mutex) < 0) {
			return ERROR_SYSTEM;
		}
		return ERROR_NOT_CALLABLE;
	}
	
	struct timespec timeout;
	clock_gettime(CLOCK_REALTIME, &timeout);
	timespec_add_msec(timeout, SEND_TIMEOUT);
	
	if(pthread_cond_timedwait(&snt_cond, &snt_mutex, &timeout) < 0) {
		if(pthread_mutex_unlock(&snt_mutex) < 0) {
			return ERROR_SYSTEM;
		}
		return ERROR_TIMEOUT;
	}
	
	
	//if(pthread_cond_wait(&snt_cond, &snt_mutex) < 0) {
	//	return ERROR_SYSTEM;
	//}
	
	if(pthread_mutex_unlock(&snt_mutex) < 0) {
		return ERROR_SYSTEM;
	}
	
	return sync_sent_status;
}

int sync_broadcast(uint8_t * buffer, int len) {
	return sync_send(BROADCAST_ADDRESS, buffer, len);
}
