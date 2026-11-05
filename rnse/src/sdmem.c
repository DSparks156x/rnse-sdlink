#include "sdmem.h"
static u32 get32(const u8 *p) { return (u32)p[0]<<24|(u32)p[1]<<16|(u32)p[2]<<8|p[3]; }
static void put32(u8 *p,u32 v) { p[0]=v>>24;p[1]=v>>16;p[2]=v>>8;p[3]=v; }
static void zero(u8 *p,u32 n) { while(n--) *p++=0; }
u32 sdmem_crc(const u8 *p,u32 n) {
    u32 c=~0u,k;
    while(n--) { c^=*p++;for(k=0;k<8;k++) c=(c>>1)^(0xEDB88320u&(0u-(c&1))); }
    return ~c;
}
static int read_range(u32 a,u32 n) {
    u32 last=a+n-1, region=a>>26;
    return n && last>=a && (last>>26)==region &&
           (region==0x20 || region==0x23 || region==0x2B);
}
static int write_range(sdmem *s,u32 a,u32 n) {
    u32 p=a&0x1FFFFFFFu, end=p+n, region=a>>26;
    if (!n || (region!=0x23 && region!=0x2B) || !read_range(a,n)) return 0;
    if (s->io.write_hi<=s->io.write_lo || p<s->io.write_lo || end>s->io.write_hi) return 0;
    return s->io.protect_hi<=s->io.protect_lo || end<=s->io.protect_lo || p>=s->io.protect_hi;
}
void sdmem_init(sdmem *s,const sdmem_io *io) {
    zero((u8 *)s,sizeof(*s));s->io=*io;
}
static void envelope(sdmem *s,u32 count) {
    put32(s->sector,0x52535550u);s->sector[4]=3;s->sector[5]=3;
    s->sector[6]=(count+24)>>8;s->sector[7]=count+24;
    put32(s->sector+8,s->seq);put32(s->sector+12,sdmem_crc(s->sector+16,count+24));
    s->pending=1;s->sent=0;s->chunk=count;
}
static void response(sdmem *s,u32 status,u32 crc) {
    u8 *p=s->sector+16;
    zero(s->sector,512);
    p[0]=1;p[1]=(u8)s->op;p[2]=(u8)status;
    put32(p+4,s->id);put32(p+8,s->addr);put32(p+12,s->length);
    put32(p+16,s->offset);put32(p+20,crc);
}
static void read_chunk(sdmem *s) {
    u32 n=s->length-s->offset;
    if(n>SDMEM_CHUNK) n=SDMEM_CHUNK;
    response(s,SDM_OK,0);
    if(!s->io.read || s->io.read(s->io.opaque,s->addr+s->offset,s->sector+40,n)) {
        s->io_errors++;response(s,SDM_IO,0);s->final=1;envelope(s,0);return;
    }
    put32(s->sector+36,sdmem_crc(s->sector+40,n));
    s->final=s->offset+n==s->length;envelope(s,n);
}
static void ack(sdmem *s,u32 seq) {
    if(!s->pending || !s->sent || seq!=s->seq) return;
    s->pending=0;
    if(!s->final) { s->offset+=s->chunk;s->seq++;read_chunk(s); }
}
static void command(sdmem *s,const u8 *p,u32 n) {
    u32 id=get32(p+4),fp=sdmem_crc(p,n),status=SDM_OK,crc=0;
    u32 op=p[1],a=get32(p+8),len=get32(p+12);
    u8 before[SDMEM_MAX_PATCH];
    if(s->have_id) {
        if(id==s->id) {
            if(fp!=s->fingerprint) { s->bad++;return; }
            /* Duplicate requests never re-execute patches or rewind a dump. */
            if(!s->pending) { s->pending=1;s->sent=0; }
            return;
        }
        if(s->pending || (s32)(id-s->id)<=0) { s->bad++;return; }
    }
    s->have_id=1;s->id=id;s->fingerprint=fp;s->op=op;s->addr=a;s->length=len;s->offset=0;s->seq++;
    s->commands++;s->final=1;
    if(p[0]!=1 || p[2] || p[3]) status=SDM_FORMAT;
    else if(op==SDM_READ) {
        if(n!=24 || get32(p+16) || get32(p+20)) status=SDM_FORMAT;
        else if(!len || len>SDMEM_MAX_READ) status=SDM_LENGTH;
        else if(!read_range(a,len)) status=SDM_RANGE;
        else { read_chunk(s);return; }
    } else if(op==SDM_PATCH) {
        if(!len || len>SDMEM_MAX_PATCH || n!=24+len) status=SDM_LENGTH;
        else if(sdmem_crc(p+24,len)!=get32(p+20)) status=SDM_CRC;
        else if(!s->io.write || s->io.write_hi<=s->io.write_lo) status=SDM_DISABLED;
        else if(!write_range(s,a,len)) status=SDM_RANGE;
        else if(!s->io.read || s->io.read(s->io.opaque,a,before,len)) status=SDM_IO;
        else if((crc=sdmem_crc(before,len))!=get32(p+16)) status=SDM_COMPARE;
        else if(s->io.write(s->io.opaque,a,p+24,len) || s->io.read(s->io.opaque,a,before,len)) status=SDM_IO;
        else if((crc=sdmem_crc(before,len))!=get32(p+20)) status=SDM_IO;
    } else status=SDM_UNSUPPORTED;
    if(status==SDM_IO) s->io_errors++;
    response(s,status,crc);envelope(s,0);
}
int sdmem_control(sdmem *s,const u8 *p,u32 n) {
    u32 i=0,commands=0;
    if(n>448) { s->bad++;return -1; }
    /* First pass prevents a truncated tail from following a successful patch. */
    while(i<n) {
        u32 t,l,j;
        if(p[i]==0) {
            for(j=i;j<n;j++) if(p[j]) { s->bad++;return -1; }
            n=i;break; /* all-zero header-sector padding, even an odd byte */
        }
        if(n-i<2) { s->bad++;return -1; }
        t=p[i];l=p[i+1];
        if(l>n-i-2 || (t==SDMEM_CMD && (l<24 || ++commands>1)) ||
           (t==SDMEM_ACK && (l!=5 || p[i+2]!=SDMEM_CHANNEL))) { s->bad++;return -1; }
        i+=2+l;
    }
    /* ACK the preceding sector before accepting a following stop-and-wait command. */
    for(i=0;i<n;i+=2+p[i+1]) if(p[i]==SDMEM_ACK) ack(s,get32(p+i+3));
    for(i=0;i<n;i+=2+p[i+1]) if(p[i]==SDMEM_CMD) command(s,p+i+2,p[i+1]);
    return 0;
}
void sdmem_pump(sdmem *s,u32 tick) {
    int retry;
    if(!s->pending || (s->sent && tick-s->sent_at<3)) return;
    retry=s->sent!=0;
    if(!s->io.send || s->io.send(s->io.opaque,SDMEM_LBA+(s->seq&0x3FFFu),s->sector)) s->io_errors++;
    /* Rate-limit I/O failures too; application ACK is the delivery authority. */
    s->sent=1;s->sent_at=tick;s->tx++;if(retry)s->retries++;
}
