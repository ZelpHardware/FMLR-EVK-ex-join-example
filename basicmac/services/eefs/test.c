#include "picofs.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#define ASSERT_EX(cond,fmt,...) do { \
    if (!(cond)) { \
        printf("\n %s:%d: assertion failed: %s" fmt "\n", \
                __FILE__, __LINE__, #cond __VA_OPT__(,) __VA_ARGS__); \
        return 1; \
    } \
} while( 0 )

#define ASSERT(cond) ASSERT_EX(cond, "")

#define TEST_BIN_1 \
    "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do " \
    "eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut " \
    "enim ad minim veniam, quis nostrud exercitation ullamco laboris " \
    "nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor " \
    "in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla " \
    "pariatur. Excepteur sint occaecat cupidatat non proident, sunt in " \
    "culpa qui officia deserunt mollit anim id est laborum."

static unsigned char EEPROM[6*1024];

void pfs_write_block (void* dst, void* src, int nwords) {
    memcpy(dst, src, nwords * 4);
}

void pfs_crc32 (uint32_t* pcrc, unsigned char* buf, uint32_t len) {
    int i;
    uint32_t byte, crc, mask;

    if( buf == NULL ) {
        static int init = 0;
        *pcrc = init++;
        return;
    }

    crc = ~(*pcrc);
    while( len-- != 0 ) {
	byte = *buf++;
	crc = crc ^ byte;
	for( i = 7; i >= 0; i-- ) {
	    mask = -(crc & 1);
	    crc = (crc >> 1) ^ (0xEDB88320 & mask);
	}
    }
    *pcrc = ~crc;
}

enum {
    INIT_ASIS,
    INIT_0,
    INIT_1,
    INIT_RND,

    MAX_INIT,
};

static void init (pfs* s, int type) {
    switch( type ) {
        case INIT_0:
            memset(EEPROM, 0x00, sizeof(EEPROM));
            break;
        case INIT_1:
            memset(EEPROM, 0xff, sizeof(EEPROM));
            break;
        case INIT_RND:
            for( int i = 0; i < sizeof(EEPROM); i += 4 ) {
                *((uint32_t*) (EEPROM + i)) = rand();
            }
            break;
        default:
            break;
    };
    pfs_init(s, EEPROM, sizeof(EEPROM) / 32);
}

static void cb_count (int fh, const uint8_t* ufid, void* ctx) {
    *((int*) ctx) += 1;
}

// test that an "empty" EEPROM (filled with 0, 1, or random bytes)
// contains zero files
int test_clean (void) {
    pfs s;
    for( int i = 0; i < MAX_INIT; i++) {
        int ct = 0;
        init(&s, i);
        pfs_ls(&s, cb_count, &ct);
        ASSERT_EX(ct == 0, " (init=%d, ct=%d)", i, ct);
    }
    return 0;
}

// test creation of a file on clean EEPROM
int test_create (void) {
    pfs s;
    for( int i = 0; i < MAX_INIT; i++) {
        int ct = 0;
        init(&s, i);

        int fd1 = pfs_save(&s, "test.file", TEST_BIN_1, strlen(TEST_BIN_1));
        ASSERT_EX(fd1 >= 0, " (init=%d, fd1=%d)", i, fd1);

        pfs_ls(&s, cb_count, &ct);
        ASSERT_EX(ct == 1, " (init=%d, ct=%d)", i, ct);

        int fd2 = pfs_find(&s, "test.file");
        ASSERT_EX(fd1 == fd2, " (init=%d, fd2=%d)", i, fd2);

        unsigned char buf[1024];
        int len = pfs_read(&s, "test.file", buf, sizeof(buf));
        ASSERT_EX(len == strlen(TEST_BIN_1), " (init=%d, len=%d)", i, len);

        int d = memcmp(TEST_BIN_1, buf, len);
        ASSERT_EX(d == 0, " (init=%d, d=%d)", i, d);
    }
    return 0;
}

int test_names (void) {
    pfs s;
    init(&s, INIT_RND);

    const char FN[5][16] = {
        { 0xb0, 0x5d, 0x86, 0x6a, 0xf4, 0x1e, 0x86, 0x16, 0xc7, 0x9e, 0xbb, 0x9d },
        { 0xc0, 0xab, 0x8f, 0x6a, 0xf4, 0x1e, 0x86, 0x16, 0x47, 0x66, 0x4e, 0x96 },
        { 0x80, 0x46, 0x88, 0x6a, 0xf4, 0x1e, 0x86, 0x16, 0xee, 0x6f, 0xae, 0xbd },
        { 0x20, 0x95, 0x8e, 0x6a, 0xf4, 0x1e, 0x86, 0x16, 0xc6, 0xa9, 0xf6, 0x08 },
        { 0xc0, 0xf5, 0x8c, 0x6a, 0xf4, 0x1e, 0x86, 0x16, 0x04, 0x5b, 0x28, 0xba }
    };
    unsigned char* FC[5] = {
        "11111",
        "22222",
        "33333",
        "44444",
        "55555"
    };

    int fd[5];
    for( int i=0; i < 5; i++ ) {
        int fd1 = pfs_save(&s, FN[i], FC[i], 5);
        ASSERT_EX(fd1 >= 0, " (i=%d, fd1=%d)", i, fd1);

        fd[i] = fd1;
        for( int j=0; j < i; j++ ) {
            ASSERT_EX(fd[j] != fd[i], " (i=%d, j=%d, fd=%d)", i, j, fd1);
        }
    }
    for( int i=0; i < 5; i++ ) {
        int fd2 = pfs_find(&s, FN[i]);
        ASSERT_EX(fd[i] == fd2, " (i=%d, fd[i]=%d, fd2=%d)", i, fd[i], fd2);

        char buf[1024];
        int len = pfs_read(&s, FN[i], buf, sizeof(buf));
        ASSERT_EX(len == 5, " (i=%d, len=%d)", i, len);

        int d = memcmp(FC[i], buf, len);
        ASSERT_EX(d == 0, " (i=%d, d=%d)", i, d);
    }
    return 0;
}

#define RUNTEST(t) do { \
    printf("%s: ", #t); \
    if( test_ ## t() == 0 ) { \
        printf(" ok\n"); \
    } else { \
        failed = 1; \
        printf(" failed\n"); \
    } \
} while( 0 )

int main (void) {
    srand(time(NULL));
    int failed = 0;

    RUNTEST(clean);
    RUNTEST(create);
    RUNTEST(names);

    return failed;
}
