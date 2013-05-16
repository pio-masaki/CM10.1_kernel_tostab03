#define LEN_MANU 				10
#define LEN_PRODUCTNUM 		12
#define LEN_PARTNUM 			15
#define LEN_SERIALNUM 			15
#define LEN_UUID 				34
#define LEN_OEMSTR 				30
#define LEN_BOARDID 			2

/* for product dmi board information */
typedef struct 
{   
    char manu[LEN_MANU+1];
    char product_number[LEN_PRODUCTNUM+1];
    char part_number[LEN_PARTNUM+1];
    char serial[LEN_SERIALNUM+1];
    char uuid[LEN_UUID+1];
    char oem_str[LEN_OEMSTR+1];

    /* 4 bytes behind reserved are used for cap sensitivity*/
} productBoardInfo;