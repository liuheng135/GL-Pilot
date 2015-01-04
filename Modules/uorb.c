/** ***************************************************************************************
  * @file    uorb.c                                                                       *
  * @author  Liu heng                                                                     *
  * @version V1.0.0                                                                       *
  * @date    28-Nov-2014                                                                  *
  * @brief   The implementation of micro object request broker for RT-Thread OS           *  
  *****************************************************************************************
  *                          example                                                      *
  *                                                                                       *
  *****************************************************************************************/

#include "uorb.h"
#include "stdio.h"
#include "string.h"
#include <rthw.h>

/**
  * @name   orb_advertise
  * @brief  advertise a orb topic
  * @param      *node: orb node
  *         structure: raw data
  * @retval the operation status, RT_EOK on OK, -RT_ERROR on error
  */
rt_err_t orb_advertise(orb_node *node,void *structure)
{
	if(node->orb_data == RT_NULL)
	{
		node->orb_data = rt_malloc(node->size);
		node->advertiser = rt_thread_self();
		node->subscriber_list = RT_NULL;
		return RT_EOK;
	}
	else
		return RT_ERROR;
}

/**
  * @name   orb_subscribe
  * @brief  subscribe a orb topic
  * @param      *node: orb node
  *             event: pointer to a rt_event,used for orb_check()
  * @retval the operation status, RT_EOK on OK, -RT_ERROR on error
  */
rt_err_t orb_subscribe(orb_node *node,rt_uint32_t *event)
{
	rt_err_t err;
	rt_uint32_t level;
	
	struct rt_event *update;
	struct orb_subscriber *ptr;
	struct orb_subscriber *new_subscriber;
	char name[RT_NAME_MAX+4] = "orb_";
	rt_thread_t self = rt_thread_self();
	
	if(node->orb_data == RT_NULL)
	{
		rt_thread_delay(10);
		if(node->orb_data == RT_NULL)return RT_ERROR;
	}
	level = rt_hw_interrupt_disable();
	node->subscriber_num++;
	//sprintf(name,"%5d",node->subscriber_num);
	strcpy(&name[4],self->name);
	update = (struct rt_event*)rt_malloc(sizeof(struct rt_event));
	err = rt_event_init(update,name,RT_IPC_FLAG_FIFO);
	if(err != RT_EOK)
	{
		rt_hw_interrupt_enable(level);
		return err;
	}
	
	*event = (rt_uint32_t)update;
	
	new_subscriber = (struct orb_subscriber*)rt_malloc(sizeof(struct orb_subscriber));
	new_subscriber->next = RT_NULL;
	strcpy(new_subscriber->subscriber,self->name);
	new_subscriber->update = update;
	
	
	if(node->subscriber_list == RT_NULL)
	{
		node->subscriber_list = new_subscriber;
	}
	else
	{
		ptr = node->subscriber_list;
		while(ptr->next != RT_NULL)
		{
			ptr = ptr->next;
		}
		ptr->next = new_subscriber;
		
	}
	rt_hw_interrupt_enable(level);
	
	return RT_EOK;
}

/**
  * @name   orb_unsubscribe
  * @brief  unsubscribe a orb topic
  * @param      *node: orb node
  *             subscriber: pointer to a thread who subscribe this topic
  * @retval the operation status, RT_EOK on OK, -RT_ERROR on error
  */
rt_err_t orb_unsubscribe(orb_node *node,rt_thread_t subscriber)
{   
	return RT_EOK;
}

/**
  * @name   orb_publish
  * @brief  publish new data to a orb topic
  * @param      *node: orb node
  *         structure: raw data
  * @retval the operation status, RT_EOK on OK, -RT_ERROR on error
  */
rt_err_t orb_publish(orb_node *node,void *structure)
{
	rt_err_t err = RT_EOK;
	rt_err_t err_tmp;
	struct orb_subscriber *ptr;
	rt_uint32_t level;
	
	level = rt_hw_interrupt_disable();
	rt_memcpy(node->orb_data,structure,node->size);
	rt_hw_interrupt_enable(level);
	
	ptr = node->subscriber_list;
	while(ptr != RT_NULL)
	{
		err_tmp = rt_event_send((ptr->update),EVENT_ORB_DATA_UPDATE);
		if(err_tmp != RT_EOK)
			err = err_tmp;
		ptr = ptr->next;
	}
	return err;
}


/**
  * @name   orb_check
  * @brief  check if a orb topic is updated 
  * @param      *update: pointer to specified event
  *             timeout: the waiting time
  * @retval the operation status, RT_EOK on OK, -RT_ERROR on error
  */
rt_err_t orb_check(rt_uint32_t *update,rt_int32_t timeout)
{
	rt_uint32_t e;
	return rt_event_recv((struct rt_event*)*update, EVENT_ORB_DATA_UPDATE,
        RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR ,
        timeout, &e);
}

/**
  * @name   orb_copy
  * @brief  copy a data after a orb topic is updated 
  * @param      *node: orb node
  *             buffer: data buffer
  * @retval the operation status, RT_EOK on OK, -RT_ERROR on error
  */
rt_err_t orb_copy(orb_node *node,void *buffer)
{
	rt_uint32_t level;
	
	level = rt_hw_interrupt_disable();
	rt_memcpy(buffer,node->orb_data,node->size);
	rt_hw_interrupt_enable(level);
	return RT_EOK;
}
