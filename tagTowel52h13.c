/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <stdlib.h>
#include "tagTowel52h13.h"

static uint64_t codedata[48714] = {
      0x0004064a19651ff1UL,
   0x0004064a53f425b6UL,
   0x0004064a8e832b7bUL,
   0x0004064ac9123140UL,
   0x0004064b03a13705UL,
   0x00040f74905a4fcbUL,
   0x00040f769f6183b8UL,
   0x00040f78ae68b7a5UL,
   0x00040f77c42ca091UL,
   0x0004064b3e303ccaUL,
   0x00040f7df1423c58UL,
   0x00040f82f98cbb46UL,
   0x00040f8db9ceca71UL,
   0x00040f7d07062544UL,
   0x00040f98b49fdf61UL,
   0x00040f9a13fa01ffUL,
   0x0004064bb34e4854UL,
   0x00040f7b328df71cUL,
   0x0004064c286c53deUL,
   0x0004064c62fb59a3UL,
   0x00040f9580cd8e9bUL,
   0x0004064b78bf428fUL,
   0x0004064beddd4e19UL,
   0x00041d60689d7117UL,
   0x0004064c9d8a5f68UL,
   0x00041d68df494690UL,
   0x00041d6c131b9756UL,
   0x00041d6f0c5ee257UL,
   0x00041d703129ff30UL,
   0x00040b1d17b7cda4UL,
   0x00040b1fd66c12e0UL,
   0x00040b23f47a7abaUL,
   0x00040b27284ccb80UL,
   0x00040b2887a6ee1eUL,
   0x00040b2971e30532UL,
   0x00040b29e70110bcUL,
   0x00040b2d1ad36182UL,
   0x00040b2e050f7896UL,
   0x00040b2e3f9e7e5bUL,
   0x000419e3d8a38061UL,
   0x000419e44dc18bebUL,
   0x000419e9909b109eUL,
   0x000419ecc46d6164UL,
   0x000419ef4892a0dbUL,
   0x000419fb2d9fccdfUL,
   0x000419e537fda2ffUL,
   0x00040b32d2caf1bfUL,
   0x0004080025454898UL,
   0x00040b33f7960e98UL,
   0x000406d7b1381348UL,
   0x000406dfed54e2fcUL,
   0x0004089a1743737dUL,
   0x0004089b3c0e9056UL,
   0x0004089bb12c9be0UL,
   0x0004089d1086be7eUL,
   0x000408a253604331UL,
   0x000408a2c87e4ebbUL,
   0x000408a3030d5480UL,
   0x000408a427d87159UL,
   0x000408a5c1c199bcUL,
   0x000413f276d89b3aUL,
   0x000413f2b167a0ffUL,
   0x000408a6e68cb695UL,
   0x000413f32685ac89UL,
   0x000413f61fc8f78aUL,
   0x000413f5358ce076UL,
   0x000408a8bb04e4bdUL,
   0x000408a8f593ea82UL,
   0x000413fc124f8d8cUL,
   0x000413fa3dd75f64UL,
   0x000410b5f34cf9fcUL,
   0x000407fa32beb296UL,
   0x000406cb570cdbbaUL,
   0x000406cbcc2ae744UL,
   0x000406cda0a3156cUL,
   0x000406cf751b4394UL,
   0x000406d1f940830bUL,
   0x000410b20fcd97e7UL,
   0x000410b41ed4cbd4UL,
   0x000410b36f27ba85UL,
   0x000410b7181816d5UL,
   0x000410b752a71c9aUL,
   0x000410b78d36225fUL,
   0x000410b9d6cc5c11UL,
   0x000410ba4bea679bUL,
   0x000406d68c6cf66fUL,
   0x000410bafb9778eaUL,
   0x000410bc9580a14dUL,
   0x000406dc446486acUL,
   0x000406dd692fa385UL,
   0x000406de8dfac05eUL,
   0x000410bac1087325UL,
   0x000406e1873e0b5fUL,
   0x000411f1d8d719b5UL,
   0x00040ce5aad8c9a0UL,
   0x00040ce70a32ec3eUL,
   0x00040ceab323488eUL,
   0x00040cedac66938fUL,
   0x00040cede6f59954UL,
   0x00040cf1ca74fb69UL,
   0x00040cf4892940a5UL,
   0x00040cf623126908UL,
   0x00040cf7bcfb916bUL,
   0x00041d7489c76ccfUL,
   0x00041d770decac46UL,
   0x00040cfaf0cde231UL,
   0x00041d836817e3d4UL,
   0x00041d86615b2ed5UL,
   0x00041d9b32323bdcUL,
   0x00041d9db6577b53UL,
   0x00041d995dba0db4UL,
   0x00040cf832199cf5UL,
   0x00040cfa7bafd6a7UL,
   0x000413e98b0eba37UL,
   0x000413ee93593925UL,
   0x000413ef087744afUL,
   0x000413f201ba8fb0UL,
   0x000413f23c499575UL,
   0x000409a78d63ffffUL,
   0x000409a99c6b33ecUL,
   0x000409a9d6fa39b1UL,
   0x000409aa11893f76UL,
   0x000409aafbc5568aUL,
   0x000409aedf44b89fUL,
   0x000409b0ee4bec8cUL,
   0x000409b24da60f2aUL,
   0x000409b337e2263eUL,
   0x000409b4221e3d52UL,
   0x000415f105d2e8d2UL,
   0x000415f17af0f45cUL,
   0x000415f2da4b16faUL,
   0x000415f5d38e61fbUL,
   0x000415f60e1d67c0UL,
   0x000415f9f19cc9d5UL,
   0x000415fcb0510f11UL,
   0x000409b45cad4317UL,
   0x000409b63125713fUL,
   0x000415e47118ab7fUL,
   0x000415e64590d9a7UL,
   0x000415e904451ee3UL,
   0x000415ee471ea396UL,
   0x000415f090b4dd48UL,
   0x000407f09747c044UL,
   0x000407f2a64ef431UL,
   0x000407f3908b0b45UL,
   0x000407f440381c94UL,
   0x000407f52a7433a8UL,
   0x000411ef54b1da3eUL,
   0x000407f64f3f5081UL,
   0x000411f1292a0866UL,
   0x000411fd48c63a2fUL,
   0x0004120166d4a209UL,
   0x000412071ecc3246UL,
   0x000412087e2654e4UL,
   0x000411f45cfc592cUL,
   0x000407f5da2144f7UL,
   0x0004120e361de521UL,
   0x00041212c94a5885UL,
   0x000407f7e92878e4UL,
   0x000407f823b77ea9UL,
   0x0004120a529e830cUL,
   0x000407fcf172f7d2UL,
   0x0004120d1152c848UL,
   0x000419cd33544532UL,
   0x000419d4fa53095cUL,
   0x000419d743e9430eUL,
   0x000419df45770cfdUL,
   0x000419e2b3d86388UL,
};
apriltag_family_t *tagTowel52h13_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tagTowel52h13");
   tf->h = 13;
   tf->ncodes = 48714;
   tf->codes = codedata;
   tf->nbits = 52;
   tf->bit_x = calloc(52, sizeof(uint32_t));
   tf->bit_y = calloc(52, sizeof(uint32_t));
   tf->bit_x[0] = -2;
   tf->bit_y[0] = -2;
   tf->bit_x[1] = -1;
   tf->bit_y[1] = -2;
   tf->bit_x[2] = 0;
   tf->bit_y[2] = -2;
   tf->bit_x[3] = 1;
   tf->bit_y[3] = -2;
   tf->bit_x[4] = 2;
   tf->bit_y[4] = -2;
   tf->bit_x[5] = 3;
   tf->bit_y[5] = -2;
   tf->bit_x[6] = 4;
   tf->bit_y[6] = -2;
   tf->bit_x[7] = 5;
   tf->bit_y[7] = -2;
   tf->bit_x[8] = 6;
   tf->bit_y[8] = -2;
   tf->bit_x[9] = 1;
   tf->bit_y[9] = 1;
   tf->bit_x[10] = 2;
   tf->bit_y[10] = 1;
   tf->bit_x[11] = 3;
   tf->bit_y[11] = 1;
   tf->bit_x[12] = 2;
   tf->bit_y[12] = 2;
   tf->bit_x[13] = 7;
   tf->bit_y[13] = -2;
   tf->bit_x[14] = 7;
   tf->bit_y[14] = -1;
   tf->bit_x[15] = 7;
   tf->bit_y[15] = 0;
   tf->bit_x[16] = 7;
   tf->bit_y[16] = 1;
   tf->bit_x[17] = 7;
   tf->bit_y[17] = 2;
   tf->bit_x[18] = 7;
   tf->bit_y[18] = 3;
   tf->bit_x[19] = 7;
   tf->bit_y[19] = 4;
   tf->bit_x[20] = 7;
   tf->bit_y[20] = 5;
   tf->bit_x[21] = 7;
   tf->bit_y[21] = 6;
   tf->bit_x[22] = 4;
   tf->bit_y[22] = 1;
   tf->bit_x[23] = 4;
   tf->bit_y[23] = 2;
   tf->bit_x[24] = 4;
   tf->bit_y[24] = 3;
   tf->bit_x[25] = 3;
   tf->bit_y[25] = 2;
   tf->bit_x[26] = 7;
   tf->bit_y[26] = 7;
   tf->bit_x[27] = 6;
   tf->bit_y[27] = 7;
   tf->bit_x[28] = 5;
   tf->bit_y[28] = 7;
   tf->bit_x[29] = 4;
   tf->bit_y[29] = 7;
   tf->bit_x[30] = 3;
   tf->bit_y[30] = 7;
   tf->bit_x[31] = 2;
   tf->bit_y[31] = 7;
   tf->bit_x[32] = 1;
   tf->bit_y[32] = 7;
   tf->bit_x[33] = 0;
   tf->bit_y[33] = 7;
   tf->bit_x[34] = -1;
   tf->bit_y[34] = 7;
   tf->bit_x[35] = 4;
   tf->bit_y[35] = 4;
   tf->bit_x[36] = 3;
   tf->bit_y[36] = 4;
   tf->bit_x[37] = 2;
   tf->bit_y[37] = 4;
   tf->bit_x[38] = 3;
   tf->bit_y[38] = 3;
   tf->bit_x[39] = -2;
   tf->bit_y[39] = 7;
   tf->bit_x[40] = -2;
   tf->bit_y[40] = 6;
   tf->bit_x[41] = -2;
   tf->bit_y[41] = 5;
   tf->bit_x[42] = -2;
   tf->bit_y[42] = 4;
   tf->bit_x[43] = -2;
   tf->bit_y[43] = 3;
   tf->bit_x[44] = -2;
   tf->bit_y[44] = 2;
   tf->bit_x[45] = -2;
   tf->bit_y[45] = 1;
   tf->bit_x[46] = -2;
   tf->bit_y[46] = 0;
   tf->bit_x[47] = -2;
   tf->bit_y[47] = -1;
   tf->bit_x[48] = 1;
   tf->bit_y[48] = 4;
   tf->bit_x[49] = 1;
   tf->bit_y[49] = 3;
   tf->bit_x[50] = 1;
   tf->bit_y[50] = 2;
   tf->bit_x[51] = 2;
   tf->bit_y[51] = 3;
   tf->width_at_border = 6;
   tf->total_width = 10;
   tf->reversed_border = true;
   return tf;
}

void tagTowel52h13_destroy(apriltag_family_t *tf)
{
   free(tf->bit_x);
   free(tf->bit_y);
   free(tf->name);
   free(tf);
}
