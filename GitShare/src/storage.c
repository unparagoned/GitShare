/**
  ******************************************************************************
  * @file    storage.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    08-April-2011
  * @brief   This file includes the Storage (DosFs) driver for the STM32100E-EVAL 
  *          demo.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
SD_Error Status2 = SD_OK;
/** @addtogroup STM32100E_EVAL_Demo
  * @{
  */

/** @defgroup STORAGE
  * @brief This file includes the Storage (DosFs) driver for the STM32100E-EVAL 
  *        demo.
  * @{
  */

/** @defgroup STORAGE_Private_Types
  * @{
  */
/**
  * @}
  */

/** @defgroup STORAGE_Private_Defines
  * @{
  */
/**
  * @}
  */

/** @defgroup STORAGE_Private_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup STORAGE_Private_Variables
  * @{
  */
uint8_t sector[SECTOR_SIZE];
VOLINFO  vi;
DIRINFO  di;
SD_CSD SD_csd;
extern const uint8_t SlidesCheck[];
/**
  * @}
  */


/** @defgroup STORAGE_Private_FunctionPrototypes
  * @{
  */
/**
  * @}
  */

/** @defgroup STORAGE_Private_Functions
  * @{
  */

/**
  * @brief  SDCARD Initialisation for DosFs
  * @param  None
  * @retval err : Error status (0=> success, 1=> fail)
  */
uint32_t Storage_Init(void)
{
  uint32_t err = 0;
  uint32_t pstart = 0;
  uint32_t psize;
  uint8_t  pactive;
  uint8_t  ptype;
  
  SD_Init();
 // SD_GetCSDRegister(&SD_csd);

  /****************** DOSFS Volume Acess ******************************/

  if (DFS_ReadSector(0, sector, 0, 1))
  {
    /* Cannot read media! */
    err = 1;
    return err;
  }
  /* The end of the last read sector contains a FAT or MBR end delimiter */
  if ((sector[0x1FE] == 0x55) & (sector[0x1FF] == 0xAA))
  {
    /* Well, this is not a MBR, but a FAT12/16 header! */
    if ((sector[0x36] == 'F') & (sector[0x37] == 'A') & (sector[0x38] == 'T'))
    {
      pstart = 0;
      pactive = 0x80;
      ptype = 0x06;
      psize = 0xFFFFFFFF;
    }
    /* Well, this is not a MBR, but a FAT32 header! */
    else if ((sector[0x52] == 'F') & (sector[0x53] == 'A') & (sector[0x54] == 'T'))
    {
      pstart = 0;
      pactive = 0x80;
      ptype = 0x06;
      psize = 0xFFFFFFFF;
    }
    else
    {
      pstart = DFS_GetPtnStart(0, sector, 0, &pactive, &ptype, &psize);
      if (pstart == 0xffffffff)
      {
        /* Cannot find first partition */
        err = 1;
        return err;
      }
    }
  }
  /* Partition 0:
     start sector: pstart "0x%-08.8lX"
    active: pactive "%-02.2hX" 
    type: ptype "%-02.2hX" 
    size: psize "%-08.8lX" */
  if (DFS_GetVolInfo(0, sector, pstart, &vi))
  {
    /* Error getting volume information */

    err = 1;
    return err;
  }
  
  di.scratch = sector;
  
  if (DFS_OpenDir(&vi, "", &di))
  {
    /* Error opening root directory */
    err = 1;
    return err;
  }

  return err;
}
/**
  * @brief  Open a file and copy its content to a buffer
  * @param  DirName: the Directory name to open
  * @param  FileName: the file name to open
  * @param  BufferAddress: A pointer to a buffer to copy the file to
  * @param  FileLen: the File lenght
  * @retval err: Error status (0=> success, 1=> fail)
  */
uint32_t Storage_OpenReadFileIcon(uint8_t *DirName, uint8_t *FileName, uint32_t BufferAddress, uint32_t *FileLen)
{
  uint32_t j = 0, lcdindex = 0, firstdata = 33, lastdata = 0;
  uint32_t err = 0;
  uint32_t number_of_clusters = 0;
  uint32_t i;
  FILEINFO fi;
  DIRINFO di;
  uint16_t *pbuffer;
  pbuffer = (uint16_t *)BufferAddress;

  /* Directory enumeration test */
  di.scratch = sector;
  if (DFS_OpenDir(&vi, "", &di))
  {
    err = 1;
  }
  if (DFS_OpenDir(&vi, DirName, &di))
  {
    err = 1;
  }
  if (DFS_OpenFile(&vi, FileName, DFS_READ, sector, &fi))
  {
    err = 1;
  }
  else
  {
    *FileLen = fi.filelen;
    number_of_clusters = fi.filelen / 512;
    if ((fi.filelen % SECTOR_SIZE) > 0)
    {
      number_of_clusters ++;
    }
  }

  /* Set GRAM write direction and BGR = 1 */
  /* I/D=00 (LCD_DIR_HORIZONTAL : decrement, LCD_DIR_VERTICAL : decrement) */
  /* AM=1 (address is updated in vertical writing direction) */
 // LCD_WriteReg(LCD_REG_3, 0x1008);
  //LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */

  if (err == 0)
  {
    for ( j = 0; j < number_of_clusters;j++)
    {
      DFS_ReadFile(&fi, sector, (uint8_t *) BufferAddress, &i, SECTOR_SIZE);
      for (lcdindex = firstdata; ((lcdindex < 256) && (lastdata < 4096)); lcdindex++, lastdata++)
      {
       // LCD_WriteRAM(pbuffer[lcdindex]);
      }
      firstdata = 0;

    }
  }

  /* Set GRAM write direction and BGR = 1 */
  /* I/D = 01 (LCD_DIR_HORIZONTAL : increment, LCD_DIR_VERTICAL : decrement) */
  /* AM = 1 (address is updated in vertical writing direction) */
  //LCD_WriteReg(LCD_REG_3, 0x1018);

  return err;
}
/**
  * @brief  Open a file and copy its content to a buffer
  * @param  DirName: the Directory name to open
  * @param  FileName: the file name to open
  * @param  BufferAddress: A pointer to a buffer to copy the file to
  * @param  FileLen: the File lenght
  * @retval err: Error status (0=> success, 1=> fail)
  */
uint32_t Storage_OpenReadFile(uint8_t *DirName, uint8_t *FileName, uint32_t BufferAddress, uint32_t *FileLen)
{
  uint32_t j = 0, lcdindex = 0, firstdata = 33, lastdata = 0;
  uint32_t err = 0;
  uint32_t number_of_clusters = 0;
  uint32_t i;
  FILEINFO fi;
  DIRINFO di;
  uint16_t *pbuffer;
  pbuffer = (uint16_t *)BufferAddress;

  /* Directory enumeration test */
  di.scratch = sector;
  if (DFS_OpenDir(&vi, "", &di))
  {
    err = 1;
  }
  if (DFS_OpenDir(&vi, DirName, &di))
  {
    err = 1;
  }
  if (DFS_OpenFile(&vi, FileName, DFS_READ, sector, &fi))
  {
    err = 1;
  }
  else
  {
    *FileLen = fi.filelen;
    number_of_clusters = fi.filelen / 512;
    if ((fi.filelen % SECTOR_SIZE) > 0)
    {
      number_of_clusters ++;
    }
  }

  /* Set GRAM write direction and BGR = 1 */
  /* I/D=00 (LCD_DIR_HORIZONTAL : decrement, LCD_DIR_VERTICAL : decrement) */
  /* AM=1 (address is updated in vertical writing direction) */


  if (err == 0)
  {
    for ( j = 0; j < number_of_clusters;j++)
    {
#ifndef FASTFAT
      DFS_ReadFile(&fi, sector, (uint8_t *) BufferAddress, &i, SECTOR_SIZE);
#else
      DFS_ReadSector(9, (uint8_t *) BufferAddress, fi.volinfo->dataarea+(fi.cluster - 2) * fi.volinfo->secperclus, 300);

     return err;
#endif
      BufferAddress+=512;
    }
  }

  /* Set GRAM write direction and BGR = 1 */
  /* I/D = 01 (LCD_DIR_HORIZONTAL : increment, LCD_DIR_VERTICAL : decrement) */
  /* AM = 1 (address is updated in vertical writing direction) */
 // LCD_WriteReg(LCD_REG_3, 0x1018);

  return err;
}

uint32_t getFileSector(uint8_t *DirName, uint8_t *FileName)
{
	 uint32_t j = 0, lcdindex = 0, firstdata = 33, lastdata = 0;
	  uint32_t err = 0;
	  uint32_t number_of_clusters = 0;
	  uint32_t i;
	  FILEINFO fi;
	  DIRINFO di;
	  uint16_t *pbuffer;

	  uint32_t *FileLen=0;

	  /* Directory enumeration test */
	  di.scratch = sector;
	  if (DFS_OpenDir(&vi, "", &di))
	  {
	    err = 1;
	  }
	  if (DFS_OpenDir(&vi, DirName, &di))
	  {
	    err = 1;
	  }
	  if (DFS_OpenFile(&vi, FileName, DFS_READ, sector, &fi))
	  {
	    err = 1;
	  }
	  else
	  {
	    //*FileLen = fi.filelen;
	    number_of_clusters = fi.filelen / 512;
	    if ((fi.filelen % SECTOR_SIZE) > 0)
	    {
	      number_of_clusters ++;
	    }
	  }


	  return (fi.volinfo->dataarea+(fi.cluster - 2) * fi.volinfo->secperclus)<<9;
}

/**
  * @brief  Open a file and copy its content to a buffer
  * @param  DirName: the Directory name to open
  * @param  FileName: the file name to open
  * @param  BufferAddress: A pointer to a buffer to copy the file to
  * @param  FileLen: the File lenght
  * @retval err: Error status (0=> success, 1=> fail)
  */
uint32_t Storage_CheckBitmapFile(uint8_t *DirName, uint8_t *FileName, uint32_t BufferAddress, uint32_t *FileLen)
{
  uint32_t err = 0;
  uint32_t number_of_clusters = 0;
  uint32_t i;
  FILEINFO fi;
  DIRINFO di;

  /* Directory enumeration test */
  di.scratch = sector;
  
  if (DFS_OpenDir(&vi, "", &di))
  {
    err = 1;
  }
  if (DFS_OpenDir(&vi, DirName, &di))
  {
    err = 2;
  }
  if (DFS_OpenFile(&vi, FileName, DFS_READ, sector, &fi))
  {
    err = 3;
  }
  else
  {
    *FileLen = fi.filelen;
    number_of_clusters = fi.filelen / 512;
    if ((fi.filelen % SECTOR_SIZE) > 0)
    {
      number_of_clusters ++;
    }
  }

  DFS_ReadFile(&fi, sector, (uint8_t *) BufferAddress, &i, SECTOR_SIZE);
  
  //if (Menu_Buffercmp((uint8_t *)SlidesCheck, (uint8_t *) BufferAddress, 6) != 0)
//  {
 //   err = 4;
 // }
  return err;
}

/**
  * @brief  Open a file and copy its content to a buffer
  * @param  DirName: the Directory name to open
  * @param  FileName: the file name to open
  * @param  BufferAddress: A pointer to a buffer to copy the file to
  * @param  FileLen: the File lenght
  * @retval err: Error status (0=> success, 1=> fail)
  */
uint32_t Storage_OpenReadFileWave(uint8_t *DirName, uint8_t *FileName, uint8_t* BufferAddress, uint32_t *FileLen)
{
  uint32_t j = 0;
  uint32_t err = 0;
  uint32_t number_of_clusters;
  uint32_t i;
  FILEINFO fi;
  DIRINFO di;

  /* Directory enumeration test */
  di.scratch = sector;
  
  if (DFS_OpenDir(&vi, "", &di))
  {
    err = 1;
  }
  if (DFS_OpenDir(&vi, DirName, &di))
  {
    err = 1;
  }
  if (DFS_OpenFile(&vi, FileName, DFS_READ, sector, &fi))
  {
    err = 1;
  }
  else
  {
    *FileLen = fi.filelen;
    number_of_clusters = fi.filelen / 512;
    if ((fi.filelen % SECTOR_SIZE) > 0)
    {
      number_of_clusters ++;
    }
  }

  if (err == 0)
  {
    for ( j = 0; j < number_of_clusters;j++)
    {
      DFS_ReadFile(&fi, sector, BufferAddress, &i, SECTOR_SIZE);

    }
  }

  return err;
}

/**
  * @brief  List up to 25 file on the root directory with extension .BMP
  * @param  None
  * @retval The number of the found files
  */
uint32_t Storage_GetDirectoryFiles (char* Files[])
{
  uint32_t i = 0;
  DIRENT   de;
  DIRINFO di;

  /* Directory enumeration test */
  di.scratch = sector;
  if (DFS_OpenDir(&vi, "", &di))
  {}
  if (DFS_OpenDir(&vi, "USER", &di))
  {}

  while (!DFS_GetNext(&vi, &di, &de))
  {
    if (de.name[0])
    {
      if (i < MAX_BMP_FILES)
      {
        if ((de.name[8] == 'B') && (de.name[9] == 'M') && (de.name[10] == 'P'))
        {
          sprintf (Files[i], "%-11.11s", de.name);
          i++;
        }
      }
    }
  }
  return i;
}

/**
  * @brief  List up to 25 file on the root directory with extension .WAV
  * @param  None
  * @retval The number of the found files
  */
uint32_t Storage_GetDirectoryWaveFiles (char* Files[])
{
  uint32_t i = 0;
  DIRENT   de;
  DIRINFO di;

  /* Directory enumeration test */
  di.scratch = sector;
  
  if (DFS_OpenDir(&vi, "", &di))
  {}
  if (DFS_OpenDir(&vi, "USER", &di))
  {}

  while (!DFS_GetNext(&vi, &di, &de))
  {
    if (de.name[0])
    {
      if (i < MAX_BMP_FILES)
      {
        if ((de.name[8] == 'W') && (de.name[9] == 'A') && (de.name[10] == 'V'))
        {
          sprintf (Files[i], "%-11.11s", de.name);
          i++;
        }
      }
    }
  }
  return i;
}

/* User-supplied functions -------------------------------------------------- */
/**
  * @brief  Read Sector (512 bytes)
  * @param  unit: the unit
  * @param  buffer: A pointer to a buffer to copy the file to
  * @param  sector: the sector
  * @param  count: the counter
  * @retval err: Error status (0=> success, 1=> fail)
  */
uint32_t DFS_ReadSector(uint8_t unit, uint8_t *buffer, uint32_t sector, uint32_t count)
{

	if(unit==9)
	{
		SD_LCDReadMultiBlocks(buffer, sector << 9, SECTOR_SIZE,count);
	}
	else
	{
		  SD_ReadMultiBlocks(buffer, sector << 9, SECTOR_SIZE,count);
	}

  Status2 = SD_WaitReadOperation();
	//Delay(100);
  while(SD_GetStatus() != SD_TRANSFER_OK);
  return 0;
}

/**
  * @brief  Write Sector (512 bytes)
  * @param  unit: the unit
  * @param  buffer: A pointer to a buffer to copy the file to
  * @param  sector: the sector
  * @param  count: the counter
  * @retval err: Error status (0=> success, 1=> fail)
  */
uint32_t DFS_WriteSector(uint8_t unit, uint8_t *buffer, uint32_t sector, uint32_t count)
{
  SD_WriteBlock(buffer, sector << 9, SECTOR_SIZE);
  Status2 = SD_WaitWriteOperation();
  while(SD_GetStatus() != SD_TRANSFER_OK);
  return 0;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
