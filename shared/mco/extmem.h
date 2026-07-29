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

#ifndef _EXTMEM_H
#define _EXTMEM_H

#ifdef __cplusplus
extern "C" {
#endif

#if USE_EXTERNAL_MEMORY

/**************************************************************************
DEFINITIONS
***************************************************************************/

typedef struct {
  uint8_t available;  // TRUE if available
  uint8_t has_store;  // TRUE if EXTMEM_Store is available
  // Pointer to the beginning of the external memory.
  // Note: All pointers to external memory have the bits in
  // EXT_MEM_MASK set to EXT_MEM_PTR so that they will not be accessible
  // in address space and direct access will lead to an exception.
  void *base;
  uint32_t size;      // Total size of ext. memory, starting at base
} extmemInfo_t;


/**************************************************************************
PUBLIC FUNCTIONS
***************************************************************************/

/**************************************************************************
DOES:    Initializes the external memory and returns extmem info.
RETURNS: Zero if successful; an error code if unsupported or failure.
         Additional information in the extmemInfo_t structure.
**************************************************************************/
int MCOEM_Init (
  extmemInfo_t *pExtmemInfo // pointer to ext. memory info structure
  );

/**************************************************************************
DOES:    Returns the size of external memory.
RETURNS: Size of the external memory.
**************************************************************************/
int MCOEM_GetSize (
  void
  );

/**************************************************************************
DOES:    Stores the external memory into NVOL if possible.
RETURNS: Zero if successful; an error code if unsupported or failure.
**************************************************************************/
int MCOEM_Store (
  void
  );

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
  );

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
  );


/**************************************************************************
DOES:    Initializes the external memory and returns extmem info.
RETURNS: Zero if successful; an error code if unsupported or failure.
         Additional information in the extmemInfo_t structure.
**************************************************************************/
int EXTMEM_Init (
  extmemInfo_t* pExtmemInfo // pointer to ext. memory info structure
);

/**************************************************************************
DOES:    Returns the size of external memory.
RETURNS: Size of the external memory.
**************************************************************************/
int EXTMEM_GetSize (
  void
  );

/**************************************************************************
DOES:    Stores the external memory into NVOL if possible.
RETURNS: Zero if successful; an error code if unsupported or failure.
**************************************************************************/
int EXTMEM_Store (
  void
  );

/**************************************************************************
DOES:    C11 conformant implementation of memset_s for external
         memory.
         Note: All pointers to external memory have the bits in
         EXT_MEM_MASK set to EXT_MEM_PTR so that they will not be accessible
         in address space and direct access will lead to an exception.
RETURNS: Zero if successful; an error code on failure.
**************************************************************************/
int EXTMEM_memset_s (
  void *dest,       // Pointer to the destination array where the content is to be copied, type-casted to a pointer of type void*.
  size_t destSize,  // Size of the destination buffer in bytes
  int c,            // The value to be set is the unsigned char conversion of this parameter.
  size_t count      // The number of bytes to be copied.
  );

/**************************************************************************
DOES:    C11 conformant implementation of memcpy_s to/from external
         memory.
         Note: All pointers to external memory have the bits in
         EXT_MEM_MASK set to EXT_MEM_PTR so that they will not be accessible
         in address space and direct access will lead to an exception.
RETURNS: Zero if successful; an error code on failure.
**************************************************************************/
int EXTMEM_memcpy_s (
  void *dest,       // Pointer to the destination array where the content is to be copied, type-casted to a pointer of type void*.
  size_t destSize,  // Size of the destination buffer in bytes
  const void * src, // Pointer to the source of data to be copied, type-casted to a pointer of type void*.
  size_t count      // The number of bytes to be copied.
  );

#endif // USE_EXTERNAL_MEMORY


#ifdef __cplusplus
}
#endif

#endif // _EXTMEM_H
/**************************************************************************
END OF FILE
**************************************************************************/
