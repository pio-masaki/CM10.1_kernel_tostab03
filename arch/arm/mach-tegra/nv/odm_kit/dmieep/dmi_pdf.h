#ifndef __DMI_PDF_H__
#define __DMI_PDF_H__

typedef struct
{
    int     byte_count;     // byte count of this structure
    char    data[56];       // available data field, total size is 52 bytes = 56 bytes - sizeof(int)

} Dmi_PassingData_Format;


#endif