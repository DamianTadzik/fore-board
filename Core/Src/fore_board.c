/*
 * fore_board.c
 *
 *  Created on: Sep 1, 2025
 *      Author: brzan
 */
#include "fore_board.h"

static fore_board_t fb;

void fore_board_init(void)
{

}

fore_board_t* fore_board_get_ptr(void)
{
	return &fb;
}
