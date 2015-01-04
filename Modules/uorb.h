/** ******************************************************************************
  * @file    uorb.h                                                              *
  * @author  Liu heng                                                            *
  * @version V1.0.0                                                              *
  * @date    28-Nov-2014                                                         *
  * @brief   Hearder file for micro object request broker                        *
  *                                                                              *
  ********************************************************************************
  *                                                                              *
  ********************************************************************************/

#ifndef _UORB_H_
#define _UORB_H_

#include "rtthread.h"
#include "stdint.h"

#define EVENT_ORB_DATA_UPDATE   0x02

#define ORB_ID(_name)		&__orb_##_name

#define ORB_DECLARE(_name) extern orb_node __orb_##_name

typedef struct orb_subscriber
{
    void *update;
	char subscriber[RT_NAME_MAX];
    struct orb_subscriber *next;
}orb_subscriber;	


typedef struct orb_node 
{
	char *name;
	uint32_t size;
	uint32_t updated_num;
	uint32_t subscriber_num;
	rt_thread_t advertiser;
	void *orb_data;
	orb_subscriber *subscriber_list;
}orb_node;
	
/**
 * Define (instantiate) the uORB metadata for a topic.
 *
 * The uORB metadata is used to help ensure that updates and
 * copies are accessing the right data.
 *
 * Note that there must be no more than one instance of this macro
 * for each topic.
 *
 * @param _name		The name of the topic.
 * @param _struct	The structure the topic provides.
 */
#define ORB_DEFINE(_name, _struct)			\
        orb_node __orb_##_name = {	                \
		#_name,					            \
		sizeof(_struct),			        \
		0,                                  \
		0,                                  \
		RT_NULL,	                            \
		RT_NULL,                             \
		RT_NULL                             \
	};

/**
  * @name   orb_advertise
  * @brief  advertise a orb topic
  * @param      *node: orb node
  *         structure: raw data
  * @retval the operation status, RT_EOK on OK, -RT_ERROR on error
  */
rt_err_t orb_advertise(orb_node *node,void *structure);
	
/**
  * @name   orb_subscribe
  * @brief  subscribe a orb topic
  * @param      *node: orb node
  *             event: pointer to a rt_event,used for orb_check()
  * @retval the operation status, RT_EOK on OK, -RT_ERROR on error
  */
rt_err_t orb_subscribe(orb_node *node,rt_uint32_t *event);

	
/**
  * @name   orb_publish
  * @brief  publish new data to a orb topic
  * @param      *node: orb node
  *         structure: raw data
  * @retval the operation status, RT_EOK on OK, -RT_ERROR on error
  */
rt_err_t orb_publish(orb_node *node,void *structure);


/**
  * @name   orb_check
  * @brief  check if a orb topic is updated 
  * @param      *update: pointer to specified event
  *             timeout: the waiting time
  * @retval the operation status, RT_EOK on OK, -RT_ERROR on error
  */
rt_err_t orb_check(rt_uint32_t *update,rt_int32_t timeout);

/**
  * @name   orb_copy
  * @brief  copy a data after a orb topic is updated 
  * @param      *node: orb node
  *             buffer: data buffer
  * @retval the operation status, RT_EOK on OK, -RT_ERROR on error
  */
rt_err_t orb_copy(orb_node *node,void *buffer);






#endif
