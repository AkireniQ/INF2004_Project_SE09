/*
 * route_map.c: barcode character -> junction action (IF-05).
 *
 * The mapping arrives at run time over MQTT (PC -> car); nothing is
 * hard-coded here.  The MQTT owner hands the raw payload to
 * s3_route_parse(); the line task asks s3_route_lookup() at each junction.
 *
 * Representation (Change Requirement 1):
 *   letter  = 1..26 (A=1), 5 bits.  0 = unset.
 *   storage = one 32-bit word: bits 4..0 forward, 9..5 uturn, 14..10 left,
 *             19..15 right (20 bits used).
 *   payload = 3 bytes, see subsystem3.h.
 * Written whole by the MQTT task, read whole by the line task: a single
 * aligned 32-bit store/load is atomic on Cortex-M, so no mutex is needed and
 * the reader can never see a half-written map.
 */
#include "subsystem3.h"


LOCAL volatile UW route_word = 0;


LOCAL UW letter_code(char c)                    /* 'a'/'A' -> 1 ... 'Z' -> 26, else 0 */
{
    if (c >= 'a' && c <= 'z') { c = (char)(c - 'a' + 'A'); }
    return (c >= 'A' && c <= 'Z') ? (UW)(c - 'A' + 1) : 0;
}


LOCAL char code_letter(UW k)
{
    return (k >= 1 && k <= 26) ? (char)('A' + k - 1) : '-';
}


LOCAL UW slot(UW w, INT i)                      /* i = 0 forward, 1 uturn, 2 left, 3 right */
{
    return (w >> (5 * i)) & 0x1F;
}


EXPORT ER s3_route_set_map(char forward, char uturn, char left, char right)
{
    UW k[4];
    UW w = 0;
    INT i, j;

    k[0] = letter_code(forward); k[1] = letter_code(uturn);
    k[2] = letter_code(left);    k[3] = letter_code(right);
    for (i = 0; i < 4; i++) {
        if (k[i] == 0) { return E_PAR; }                        /* must be A-Z */
        for (j = 0; j < i; j++) {
            if (k[j] == k[i]) { return E_PAR; }                 /* must be distinct */
        }
        w |= k[i] << (5 * i);
    }
    route_word = w;                                             /* single atomic store */
    tm_printf((UB*)"[S3] route map: forward=%c uturn=%c left=%c right=%c\n",
              code_letter(k[0]), code_letter(k[1]), code_letter(k[2]), code_letter(k[3]));
    return E_OK;
}


EXPORT BOOL s3_route_valid(void)
{
    return route_word != 0;
}


EXPORT void s3_route_get(char out4[4])
{
    UW  w = route_word;                                         /* single atomic load */
    INT i;
    for (i = 0; i < 4; i++) { out4[i] = code_letter(slot(w, i)); }
}


EXPORT S3_ACTION s3_route_lookup(char code)
{
    UW w = route_word;
    UW k = letter_code(code);
    if (w == 0 || k == 0) { return ACT_NONE; }
    if (k == slot(w, 0)) { return ACT_STRAIGHT; }
    if (k == slot(w, 1)) { return ACT_UTURN; }
    if (k == slot(w, 2)) { return ACT_LEFT; }
    if (k == slot(w, 3)) { return ACT_RIGHT; }
    return ACT_NONE;
}


EXPORT const char *s3_action_name(S3_ACTION a)
{
    switch (a) {
    case ACT_STRAIGHT: return "FORWARD";
    case ACT_UTURN:    return "U-TURN";
    case ACT_LEFT:     return "LEFT";
    case ACT_RIGHT:    return "RIGHT";
    default:           return "NONE";
    }
}


/* ---- compact binary payload (primary) --------------------------------
 * 24 bits big-endian: F(5) U(5) L(5) R(5) check(4), check = F^U^L^R & 0xF.
 */
EXPORT INT s3_route_encode(char forward, char uturn, char left, char right, UB out[S3_ROUTE_BIN_LEN])
{
    UW f = letter_code(forward), u = letter_code(uturn), l = letter_code(left), r = letter_code(right);
    UW v;
    if (!f || !u || !l || !r) { return 0; }
    v = (f << 19) | (u << 14) | (l << 9) | (r << 4) | ((f ^ u ^ l ^ r) & 0xF);
    out[0] = (UB)(v >> 16); out[1] = (UB)(v >> 8); out[2] = (UB)v;
    return S3_ROUTE_BIN_LEN;
}


LOCAL ER parse_binary(const UB *p)
{
    UW v = ((UW)p[0] << 16) | ((UW)p[1] << 8) | p[2];
    UW f = (v >> 19) & 0x1F, u = (v >> 14) & 0x1F, l = (v >> 9) & 0x1F, r = (v >> 4) & 0x1F;
    if (((f ^ u ^ l ^ r) & 0xF) != (v & 0xF)) { return E_PAR; }         /* check nibble */
    return s3_route_set_map(code_letter(f), code_letter(u), code_letter(l), code_letter(r));
}


/* ---- text payload (debug fallback) ------------------------------------
 * Words = runs of letters.  With ':' or '=' present it is key/value: a
 * keyword opens a slot, the next single-letter word fills it.  Otherwise
 * exactly four letters in the order forward, uturn, left, right.
 */
LOCAL char up(char c) { return (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c; }
LOCAL BOOL is_letter(char c) { return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'); }

LOCAL BOOL word_eq(const UB *p, INT n, const char *kw)
{
    INT i;
    for (i = 0; i < n; i++) {
        if (kw[i] == 0 || up((char)p[i]) != up(kw[i])) { return FALSE; }
    }
    return kw[n] == 0;
}

LOCAL INT keyword_slot(const UB *p, INT n)      /* -1 = not a keyword */
{
    static const char *kw[4][4] = {
        { "forward", "fwd", "straight", "f" },
        { "uturn",   "u",   "back",     "turn" },   /* "u-turn" tokenises as "u","turn" */
        { "left",    "l",   0,          0 },
        { "right",   "r",   0,          0 },
    };
    INT s, a;
    for (s = 0; s < 4; s++) {
        for (a = 0; a < 4 && kw[s][a]; a++) {
            if (word_eq(p, n, kw[s][a])) { return s; }
        }
    }
    return -1;
}

LOCAL ER parse_text(const UB *payload, INT len)
{
    char m[4] = { 0, 0, 0, 0 };
    BOOL keyed = FALSE;
    INT  i, n, sl = -1, nbare = 0;

    for (i = 0; i < len; i++) {
        if (payload[i] == ':' || payload[i] == '=') { keyed = TRUE; break; }
    }
    i = 0;
    while (i < len) {
        if (!is_letter((char)payload[i])) { i++; continue; }
        n = 0;
        while (i + n < len && is_letter((char)payload[i + n])) { n++; }
        if (keyed) {
            INT s = keyword_slot(payload + i, n);
            if (s >= 0 && !(n == 1 && sl >= 0)) { sl = s; }
            else if (n == 1 && sl >= 0)         { m[sl] = up((char)payload[i]); sl = -1; }
        } else {
            INT k;
            for (k = 0; k < n; k++) {
                if (nbare < 4) { m[nbare] = up((char)payload[i + k]); }
                nbare++;
            }
        }
        i += n;
    }
    if (!keyed && nbare != 4) { return E_PAR; }
    return s3_route_set_map(m[0], m[1], m[2], m[3]);
}


EXPORT ER s3_route_parse(const UB *payload, INT len)
{
    if (payload == 0 || len <= 0) { return E_PAR; }
    if (len == S3_ROUTE_BIN_LEN) { return parse_binary(payload); }
    return parse_text(payload, len);
}