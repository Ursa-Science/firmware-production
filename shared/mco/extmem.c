/**************************************************************************
MODULE:    EXTMEM
CONTAINS:  MicroCANopen Plus implementation, Off-chip Memory Function
COPYRIGHT: Embedded Systems Academy (EmSA) 2002-2024
           All rights reserved. esacademy.com
DISCLAIM:  Read and understand our disclaimer before using this code!
           www.esacademy.com/disclaim.htm
           This software was written in accordance to the guidelines at
           www.esacademy.com/software/softwarestyleguide.pdf
LICENSE:   THIS IS THE COMMERCIAL PLUS VERSION OF MICROCANOPEN
           ONLY USERS WHO PURCHASED A LICENSE MAY USE THIS SOFTWARE
           See file license_commercial_plus.txt
VERSION:   7.17, EmSA 04-MAR-24
           $LastChangedDate: 2024-03-04 12:55:21 +0100 (Mon, 04 Mar 2024) $
           $LastChangedRevision: 5559 $
***************************************************************************/

#include "mcop_inc.h"

#if USE_EXTERNAL_MEMORY


/**************************************************************************
PRIVATE FUNCTIONS
***************************************************************************/


/**************************************************************************
PUBLIC FUNCTIONS
***************************************************************************/

/**************************************************************************
DOES:    Initializes the external memory and returns extmem info.
RETURNS: Zero if successful; an error code if unsupported or failure.
         Additional information in the extmemInfo_t structure.
**************************************************************************/
int MCOEM_Init (
  extmemInfo_t *pExtmemInfo
  )
{
  return EXTMEM_Init(pExtmemInfo);
}

/**************************************************************************
DOES:    Returns the size of external memory.
RETURNS: Size of the external memory.
**************************************************************************/
int MCOEM_GetSize (
  void
  )
{
  return EXTMEM_GetSize();
}

/**************************************************************************
DOES:    Stores the external memory into NVOL if possible.
RETURNS: Zero if successful; an error code if unsupported or failure.
**************************************************************************/
int MCOEM_Store (
  void
  )
{
  return EXTMEM_Store();
}

/**************************************************************************
DOES:    C11 conformant implementation of memset_s for external
         memory.
         Note: All pointers to external memory have the bits in
         EXT_MEM_MASK set to EXT_MEM_PTR so that they will not be accessible
         in address space and direct access will lead to an exception.
RETURNS: Zero if successful; an error code on failure.
**************************************************************************/
int MCOEM_memset_s (
  void *dest,       // Pointer to the destination array where the content is to be copied, type-casted to a pointer of type void*.
  size_t destSize,  // Size of the destination buffer in bytes
  int c,            // The value to be set is the unsigned char conversion of this parameter.
  size_t count      // The number of bytes to be copied.
  )
{
  return EXTMEM_memset_s(dest,destSize,c,count);
}

/**************************************************************************
DOES:    C11 conformant implementation of memcpy_s to/from external
         memory.
         Note: All pointers to external memory have the bits in
         EXT_MEM_MASK set to EXT_MEM_PTR so that they will not be accessible
         in address space and direct access will lead to an exception.
RETURNS: Zero if successful; an error code on failure.
**************************************************************************/
int MCOEM_memcpy_s (
  void *dest,       // Pointer to the destination array where the content is to be copied, type-casted to a pointer of type void*.
  size_t destSize,  // Size of the destination buffer in bytes
  const void * src, // Pointer to the source of data to be copied, type-casted to a pointer of type void*.
  size_t count      // The number of bytes to be copied.
  )
{
  return EXTMEM_memcpy_s(dest,destSize,src,count);
}

#endif // USE_EXTERNAL_MEMORY


/*----------------------- END OF FILE ----------------------------------*/
