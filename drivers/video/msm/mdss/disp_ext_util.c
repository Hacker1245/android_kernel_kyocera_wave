/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 *
 * drivers/video/msm/mdss/disp_ext_util.c
 *
 * Copyright (c) 2008-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/msm_mdp.h>
#include "disp_ext.h"
#include "mdss_dsi.h"

static int fb_dm_flag = 0;

void disp_ext_set_dmflag(int dmflag)
{
	fb_dm_flag = dmflag;
}

int disp_ext_is_invalid()
{
	if (fb_dm_flag != 0 && strcmp(current->comm, "kdispdiag") != 0) {
		return 1;
	} else {
		return 0;
	}
}

#ifdef CONFIG_DISP_EXT_PROPERTY
static void update_dsi_cmd(struct dsi_panel_cmds *cmds,
	uint8_t dtype, uint8_t cmd_addr, uint8_t *data, int len)
{
	int i;

	for (i = 0; i < cmds->cmd_cnt; i++) {
		if (cmds->cmds[i].dchdr.dtype == dtype
		&& cmds->cmds[i].payload[0] == cmd_addr) {
			if (cmds->cmds[i].dchdr.dlen == 1 + len) {
				DISP_LOCAL_LOG_EMERG("%s: %02x found at %d\n", __func__, cmd_addr, i);
				memcpy(&cmds->cmds[i].payload[1], data, len);
			}
		}
	}
}

void disp_ext_util_set_kcjprop(struct mdss_panel_data *pdata
                                       , struct fb_kcjprop_data* kcjprop_data)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	DISP_LOCAL_LOG_EMERG("DISP disp_ext_util_set_kcjprop S\n");

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	if (kcjprop_data->rw_display_gamma_valid == 0) {
		if (kcjprop_data->rw_display_gamma_normal_on_off) {
			update_dsi_cmd(&ctrl->on2_cmds, DTYPE_DCS_LWRITE, 0xE1,
				kcjprop_data->rw_display_gamma_gmct22_p, 16);
			update_dsi_cmd(&ctrl->on2_cmds, DTYPE_DCS_LWRITE, 0xE2,
				kcjprop_data->rw_display_gamma_gmct22_n, 16);
		}
		DISP_LOCAL_LOG_EMERG("%s:gamma ext set\n", __func__);
	}

	if (kcjprop_data->rw_display_cabc_valid == 0) {
		update_dsi_cmd(&ctrl->on2_cmds, DTYPE_DCS_WRITE1, 0x55,
			&kcjprop_data->rw_display_cabc, 1);
		DISP_LOCAL_LOG_EMERG("%s:cabc ext set\n", __func__);
	}

#ifdef CONFIG_DISP_EXT_DIAG
	if (kcjprop_data->rw_display_mipi_err_valid == 0) {
		if (kcjprop_data->rw_display_mipi_err == 1){
			disp_ext_diag_set_mipi_err_chk(1);
			DISP_LOCAL_LOG_EMERG("%s:mipi_err_chk_flg ON\n", __func__);
		} else if (kcjprop_data->rw_display_mipi_err == 0){
			disp_ext_diag_set_mipi_err_chk(0);
			DISP_LOCAL_LOG_EMERG("%s:mipi_err_chk_flg OFF\n", __func__);
		} else {
			DISP_LOCAL_LOG_EMERG("%s:rw_display_mipi_err invalid value \n", __func__);
		}
	}
#endif /* CONFIG_DISP_EXT_DIAG */

	DISP_LOCAL_LOG_EMERG("DISP disp_ext_util_set_kcjprop E\n");
}
#endif /* CONFIG_DISP_EXT_PROPERTY */


void disp_ext_util_set_invert_color(struct msm_fb_data_type *mfd, bool invert_color)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	pr_notice("%s: invert_color = %d\n", __func__, invert_color);

	pdata = dev_get_platdata(&mfd->pdev->dev);
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	ctrl->invert_color = invert_color;

	if (ctrl->panel_data.panel_info.cont_splash_enabled) {
		pr_info("%s: during continuous splash\n", __func__);
		return;
	}

	if (mfd->shutdown_pending || !mfd->panel_power_state) {
		pr_err("%s: panel off\n", __func__);
		return;
	}

	disp_ext_util_send_inversion_cmd(ctrl);
}

#ifdef CONFIG_DISP_EXT_ACCCESSIBILITY
#define CMD_INV_OFF 0x20
#define CMD_INV_ON  0x21

static char cmd_inv[] = {CMD_INV_OFF};
static struct dsi_cmd_desc inversion_cmd = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 1, sizeof(cmd_inv)},
	cmd_inv
};
#endif /* CONFIG_DISP_EXT_ACCCESSIBILITY */

void disp_ext_util_send_inversion_cmd(struct mdss_dsi_ctrl_pdata *ctrl)
{
#ifdef CONFIG_DISP_EXT_ACCCESSIBILITY
	struct dcs_cmd_req cmdreq;

	pr_debug("%s: invert_color=%d\n", __func__, ctrl->invert_color);

	cmd_inv[0] = ctrl->invert_color ? CMD_INV_ON : CMD_INV_OFF;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &inversion_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
#endif /* CONFIG_DISP_EXT_ACCCESSIBILITY */
}

#define OTP_CHECK_RETRY 5

static char offset_cmd[] = {0x00, 0x03};
static struct dsi_cmd_desc dcs_offset_cmd = {
	{DTYPE_DCS_WRITE1, 1, 0, 1, 0, sizeof(offset_cmd)},
	offset_cmd
};

static char otp_read_cmd[] = {0xF1};
static struct dsi_cmd_desc dcs_otp_read_cmd = {
	{DTYPE_DCS_READ, 1, 0, 1, 0, sizeof(otp_read_cmd)},
	otp_read_cmd
};

static bool otp_check_done = false;
static int otp_check = 0;

int disp_ext_otp_check(struct mdss_dsi_ctrl_pdata *ctrl)
{
	char rbuf[4];
	int retry, ret;
	struct dcs_cmd_req cmdreq;

	if (otp_check_done)
		goto exit;

	if (disp_ext_get_panel_id() == 0) {
		for (retry = 0; retry < OTP_CHECK_RETRY; retry++) {
			memset(&cmdreq, 0, sizeof(cmdreq));
			cmdreq.cmds = &dcs_offset_cmd;
			cmdreq.cmds_cnt = 1;
			cmdreq.flags = CMD_REQ_COMMIT;
			ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);
			if (ret <= 0) {
				pr_err("Disp set offset cmd failed:%d\n", ret);
				continue;
			}

			memset(&cmdreq, 0, sizeof(cmdreq));
			cmdreq.cmds = &dcs_otp_read_cmd;
			cmdreq.cmds_cnt = 1;
			cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
			cmdreq.rlen = 1;
			cmdreq.rbuf = rbuf;
			memset(rbuf, 0, sizeof(rbuf) );
			ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);
			if (ret <= 0) {
				pr_err("Disp read otp cmd failed:%d\n", ret);
			} else {
				pr_notice("%s: OTP reg:%02X\n", __func__, rbuf[0]);
				break;
			}
		}
		if (retry >= OTP_CHECK_RETRY)
			goto exit;

		if (rbuf[0] == 0x10 || rbuf[0] == 0x30 ||
				rbuf[0] == 0x70 || rbuf[0] == 0xF0)
			otp_check = 1;
	}

	otp_check_done = true;

exit:
	return otp_check;
}

int disp_ext_otp_check_value(void)
{
	if (!otp_check_done)
		return -1;

	return otp_check;
}

/*@OEMDISPLAY@_S*/
#ifdef CONFIG_DISP_EXT_KSPLASH
#define MAIN_LCD_START_X                        0                                
#define MAIN_LCD_START_Y                        0                                
#define MAIN_LCD_WIDTH                          540                              
#define MAIN_LCD_HIGHT                          960                              
#define MAIN_LCD_END_X                          (MAIN_LCD_START_X+MAIN_LCD_WIDTH)
#define MAIN_LCD_END_Y                          (MAIN_LCD_START_Y+MAIN_LCD_HIGHT)

#define SR_ID_POS 0
#define SR_BIT_DEPTH_POS 2
#define SR_TRANS_FLG_POS 3
#define SR_BYTE_PER_DOT_POS 4
#define SR_PACK_FLG_POS 5
#define SR_IMAGE_WIDTH_POS 6
#define SR_IMAGE_HEIGHT_POS 8
#define SR_TP_COLOR_POS 12
#define SR_PALET_NUM_POS 13
#define SR_TRANS_INDEX_POS 14
#define SR_NO_TRANS 0
#define SR_TRANS 1
#define SR_TRANS_ALPHA 2
#define SR_char_ORDER_MASK 0x01
#define SR_LITTLE_ENDIAN 0x00
#define SR_BIG_ENDIAN 0x01
#define SR_PACK_DIRECTION_MASK 0x02
#define SR_RIGHT_PACK 0x00
#define SR_LEFT_PACK 0x02
#define SR_PACK_LENGTH_MASK 0x04

#define SR_HEADER_SIZE 16

#define BOOT_XSTA_LIMIT(x,left)  (((x) < (left))? (left-x) : (0))
#define BOOT_YPOS_LIMIT(y,top)   (((top) > (y))?  (top-y)  : (0))
#define BOOT_XEND_LIMIT(x,width,right)   (((x+width-1) > (right))?   (right-x)  : (width-1))
#define BOOT_YEND_LIMIT(y,height,bottom) (((y+height-1) > (bottom))? (bottom-y) : (height-1))
#define BOOT_YPOS_ADD(ypos,y,column_size,x) (((ypos) + (y)) * (column_size) + (x))

short work_buf[MAIN_LCD_WIDTH];
static void rgb565_to_rgb888(short rgb565, char *out_buf)
{
  char r,g,b;
  r = (rgb565 & 0xF800) >> 8;
  g = (rgb565 & 0x07E0) >> 3;
  b = (rgb565 & 0x001F) << 3;
  out_buf[0] = b;
  out_buf[1] = g;
  out_buf[2] = r;
}

void  kernel_disp_raw_bitmap_SR(int x,
                                int y,
                                int column_size,
                                int line_size,
                                const unsigned char *imageData,
                                char *out_dbuf,
                                int twice)
{
  int width,height;
  int cnt;
  int copy_size=0;
  int xsta;
  int xend;
  int xpos;
  int ypos;
  int yend;
  int top,bottom,left,right;
  int ypos_add;
  int image_ix,work_buf_ix,ix;
  int temp_width1, temp_width2;
  char   *output_dbuf = (char *)out_dbuf;

  if(!imageData)return;

  top    = 0;
  left   = 0;
  bottom = line_size   - 1;
  right  = column_size - 1;

  if( !memcmp(imageData,"SR",2) )
  {
    short image_work;
    const char *img_data_Bp = (const char *)imageData;
    short dotParchar;

    dotParchar = imageData[SR_BYTE_PER_DOT_POS];
    width  = (unsigned int)imageData[SR_IMAGE_WIDTH_POS]  + (unsigned int)imageData[SR_IMAGE_WIDTH_POS  + 1] * 256;
    height = (unsigned int)imageData[SR_IMAGE_HEIGHT_POS] + (unsigned int)imageData[SR_IMAGE_HEIGHT_POS + 1] * 256;

    image_ix  = SR_HEADER_SIZE;
    xsta = BOOT_XSTA_LIMIT(x,left);
    xend = BOOT_XEND_LIMIT(x,width,right);
    ypos = BOOT_YPOS_LIMIT(y,top);
    yend = BOOT_YEND_LIMIT(y,height,bottom);
    ypos_add = BOOT_YPOS_ADD(ypos,y,column_size,x);
    copy_size = (xend-xsta+1)*2;
  
    temp_width1 = (width * 2) + 1;
    temp_width2 = width * 2;

    if(dotParchar == 2)
    {
      for( ;ypos <=yend; ypos++)
      {
        if(imageData[image_ix])
        {
            image_ix++;
            work_buf_ix = 0;
            while(1)
            {
              cnt = imageData[image_ix];
              image_ix++;
              image_work = (short)*(img_data_Bp+image_ix)+ (short)(*(img_data_Bp+image_ix+1) << 8 );
              for(ix = 0;ix < cnt;ix++)
              {
                work_buf[work_buf_ix + ix] = image_work;
              }
              image_ix += 2;
              work_buf_ix += cnt;
              if(work_buf_ix >= width)
              {
                break;
              }
            }
        }
        else
        {
            memcpy((char *)work_buf,&imageData[image_ix + 1], temp_width2 );
            image_ix = image_ix + temp_width1;
        }

        for( xpos=xsta; xpos<=xend; xpos++ )
        {
          if(twice){
            rgb565_to_rgb888(work_buf[xpos], (char *)&output_dbuf[ (ypos_add * 2                +  xpos * 2     ) * 3]);
            rgb565_to_rgb888(work_buf[xpos], (char *)&output_dbuf[ (ypos_add * 2                + (xpos * 2 + 1)) * 3]);
            rgb565_to_rgb888(work_buf[xpos], (char *)&output_dbuf[((ypos_add * 2 + column_size) +  xpos * 2     ) * 3]);
            rgb565_to_rgb888(work_buf[xpos], (char *)&output_dbuf[((ypos_add * 2 + column_size) + (xpos * 2 + 1)) * 3]);
          }else{
            rgb565_to_rgb888(work_buf[xpos], (char *)&output_dbuf[(ypos_add + xpos) * 3]);
          }
        }
        ypos_add += column_size;
      }
    }
  }
}
#endif
/*@OEMDISPLAY@_E*/
