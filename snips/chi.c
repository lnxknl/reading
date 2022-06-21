//==

static uint64_t fnv_hash(char *s, int len) {
  uint64_t hash = 0xcbf29ce484222325;
  for (int i = 0; i < len; i++) {
    hash *= 0x100000001b3;
    hash ^= (unsigned char)s[i];
  }
  return hash;
}

}

// Make room for new entires in a given hashmap by removing
// tombstones and possibly extending the bucket size.
static void rehash(HashMap *map) {
  // Compute the size of the new hashmap.
  int nkeys = 0;
  for (int i = 0; i < map->capacity; i++)
    if (map->buckets[i].key && map->buckets[i].key != TOMBSTONE)
      nkeys++;

  int cap = map->capacity;
  while ((nkeys * 100) / cap >= LOW_WATERMARK)
    cap = cap * 2;
  assert(cap > 0);

  // Create a new hashmap and copy all key-values.
  HashMap map2 = {};
  map2.buckets = calloc(cap, sizeof(HashEntry));
  map2.capacity = cap;

  for (int i = 0; i < map->capacity; i++) {
    HashEntry *ent = &map->buckets[i];
    if (ent->key && ent->key != TOMBSTONE)
      hashmap_put2(&map2, ent->key, ent->keylen, ent->val);
  }

  assert(map2.used == nkeys);
  *map = map2;
}
struct Token {
  TokenKind kind;   // Token kind
  Token *next;      // Next token
  int64_t val;      // If kind is TK_NUM, its value
  long double fval; // If kind is TK_NUM, its value
  char *loc;        // Token location
  int len;          // Token length
  Type *ty;         // Used if TK_NUM or TK_STR
  char *str;        // String literal contents including terminating '\0'

  File *file;       // Source location
  char *filename;   // Filename
  int line_no;      // Line number
  int line_delta;   // Line number
  bool at_bol;      // True if this token is at beginning of line
  bool has_space;   // True if this token follows a space character
  Hideset *hideset; // For macro expansion
  Token *origin;    // If this is expanded from a macro, the original token
};

typedef struct {
  HashEntry *buckets;
  int capacity;
  int used;
} HashMap;

typedef struct {
  char *key;
  int keylen;
  void *val;
} HashEntry;

void hashmap_delete(HashMap *map, char *key) {
  hashmap_delete2(map, key, strlen(key));
}

//===
bool file_exists(char *path) {
  struct stat st;
  return !stat(path, &st);
}

#ifndef _BITS_STRUCT_STAT_H
#define _BITS_STRUCT_STAT_H	1

struct stat
  {
#ifdef __USE_TIME_BITS64
# include <bits/struct_stat_time64_helper.h>
#else
    __dev_t st_dev;		/* Device.  */
# ifndef __x86_64__
    unsigned short int __pad1;
# endif
# if defined __x86_64__ || !defined __USE_FILE_OFFSET64
    __ino_t st_ino;		/* File serial number.	*/
# else
    __ino_t __st_ino;			/* 32bit file serial number.	*/
# endif
# ifndef __x86_64__
    __mode_t st_mode;			/* File mode.  */
    __nlink_t st_nlink;			/* Link count.  */
# else
    __nlink_t st_nlink;		/* Link count.  */
    __mode_t st_mode;		/* File mode.  */
# endif
    __uid_t st_uid;		/* User ID of the file's owner.	*/
    __gid_t st_gid;		/* Group ID of the file's group.*/
# ifdef __x86_64__
    int __pad0;
# endif
    __dev_t st_rdev;		/* Device number, if device.  */
# ifndef __x86_64__
    unsigned short int __pad2;
# endif
# if defined __x86_64__ || !defined __USE_FILE_OFFSET64
    __off_t st_size;			/* Size of file, in bytes.  */
# else
    __off64_t st_size;			/* Size of file, in bytes.  */
# endif
    __blksize_t st_blksize;	/* Optimal block size for I/O.  */
# if defined __x86_64__  || !defined __USE_FILE_OFFSET64
    __blkcnt_t st_blocks;		/* Number 512-byte blocks allocated. */
# else
    __blkcnt64_t st_blocks;		/* Number 512-byte blocks allocated. */
# endif
# ifdef __USE_XOPEN2K8
    /* Nanosecond resolution timestamps are stored in a format
       equivalent to 'struct timespec'.  This is the type used
       whenever possible but the Unix namespace rules do not allow the
       identifier 'timespec' to appear in the <sys/stat.h> header.
       Therefore we have to handle the use of this header in strictly
       standard-compliant sources special.  */
    struct timespec st_atim;		/* Time of last access.  */
    struct timespec st_mtim;		/* Time of last modification.  */
    struct timespec st_ctim;		/* Time of last status change.  */
#  define st_atime st_atim.tv_sec	/* Backward compatibility.  */
#  define st_mtime st_mtim.tv_sec
#  define st_ctime st_ctim.tv_sec
# else
    __time_t st_atime;			/* Time of last access.  */
    __syscall_ulong_t st_atimensec;	/* Nscecs of last access.  */
    __time_t st_mtime;			/* Time of last modification.  */
    __syscall_ulong_t st_mtimensec;	/* Nsecs of last modification.  */
    __time_t st_ctime;			/* Time of last status change.  */
    __syscall_ulong_t st_ctimensec;	/* Nsecs of last status change.  */
# endif
# ifdef __x86_64__
    __syscall_slong_t __glibc_reserved[3];
# else
#  ifndef __USE_FILE_OFFSET64
    unsigned long int __glibc_reserved4;
    unsigned long int __glibc_reserved5;
#  else
    __ino64_t st_ino;			/* File serial number.	*/
#  endif
# endif
#endif /* __USE_TIME_BITS64  */
  };

static Macro *add_macro(char *name, bool is_objlike, Token *body) {
  Macro *m = calloc(1, sizeof(Macro));
  m->name = name;
  m->is_objlike = is_objlike;
  m->body = body;
  hashmap_put(&macros, name, m);
  return m;
}

