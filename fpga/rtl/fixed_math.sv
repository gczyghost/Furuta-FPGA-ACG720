package fixed_math;
    // Signed Q15.16, SI units. 64-bit intermediates, saturating operations.
    localparam signed [31:0] ONE=65536, PI=205887, TAU=411775;
    function automatic signed [31:0] sat64(input signed [63:0] x);
      if(x>2147483647) sat64=32'sh7fffffff;
      else if(x< -64'sd2147483648) sat64=32'sh80000000;
      else sat64=x[31:0];
    endfunction
    function automatic signed [31:0] mul(input signed [31:0] a,b);
      reg signed [63:0] p;begin p=a*b;mul=sat64(p>>>16);end
    endfunction
    function automatic signed [31:0] add(input signed [31:0] a,b);
      add=sat64(64'(a)+64'(b));
    endfunction
    // Constant reciprocal multiply. factor is positive Q0.32; used instead
    // of large combinational constant dividers in the 50 MHz datapath.
    function automatic signed [31:0] ratio32(input signed [31:0] a,factor);
      reg signed [63:0] p;begin p=a*factor;ratio32=p>>>32;end
    endfunction
    function automatic signed [31:0] clip(input signed [31:0] x,lo,hi);
      clip=x<lo ? lo:x>hi ? hi:x;
    endfunction
    function automatic signed [31:0] absq(input signed [31:0] x);
      absq=x==32'sh80000000 ? 32'sh7fffffff:x<0 ? -x:x;
    endfunction
    // Exact absq(x)>limit without a two's-complement increment on x.
    // Retain saturated INT_MIN and signed negative-limit semantics.
    function automatic abs_gt(input signed [31:0] x,limit);
      abs_gt=limit<0 || (x[31] ?
        ($unsigned(~x)>=$unsigned(limit) && !(x==32'sh80000000 && limit==32'sh7fffffff)):
        x>limit);
    endfunction
    // Select the signed operand before the sole saturating absolute value.
    // Opposite-sign comparisons use one's complement, avoiding the carry
    // increment before the comparison. Ties may select either magnitude.
    function automatic signed [31:0] max_abs(input signed [31:0] x,y);
      reg choose_x;
      begin
        if(x[31]==y[31])choose_x=x[31] ? x<y:x>y;
        else if(x[31])choose_x=$unsigned(~x)>=$unsigned(y);
        else choose_x=$unsigned(x)>$unsigned(~y);
        max_abs=absq(choose_x?x:y);
      end
    endfunction
    // Q16 periodic phase: [-205887,205887], exactly TAU integer values.
    // +205887/65536 is below real pi and must retain its positive direction.
    // This is NOT the FAST command's deliberate pi+1-LSB tie policy.
    // Full reduction also makes normalization idempotent outside one turn.
    function automatic signed [31:0] wrap(input signed [31:0] x);
      // Exact reciprocal quotient estimate, followed by one correction.
      // 10430=floor(2^32/411775). For magnitude<=2^31 the quotient
      // estimate is never high and is at most one low. No signed divide.
      reg [31:0] magnitude,multiple,remainder_mag;
      reg [45:0] reciprocal_product;
      reg [13:0] quotient;
      reg signed [31:0] r;
      begin
        magnitude=x[31] ? -$unsigned(x):$unsigned(x);
        reciprocal_product=magnitude*14'd10430;
        quotient=reciprocal_product[45:32];
        multiple=quotient*19'd411775;
        remainder_mag=magnitude-multiple;
        if(remainder_mag>=32'd411775)remainder_mag=remainder_mag-32'd411775;
        r=x[31] ? -$signed(remainder_mag):$signed(remainder_mag);
        wrap=r>PI ? r-TAU:r< -PI ? r+TAU:r;
      end
    endfunction
    // Equivalent to absq(wrap(alpha-PI))<=limit for 0<=limit<PI.
    // Parallel intervals avoid subtract/wrap/negate ahead of protection.
    function automatic bit upright_le(input signed [31:0] alpha,limit);
      upright_le=(alpha>=PI-limit&&alpha<=PI+limit)||
        (alpha>=PI-TAU-limit&&alpha<=PI-TAU+limit)||
        (alpha>=PI+TAU-limit&&alpha<=PI+TAU+limit);
    endfunction
    function automatic bit upright_lt(input signed [31:0] alpha,limit);
      upright_lt=(alpha>PI-limit&&alpha<PI+limit)||
        (alpha>PI-TAU-limit&&alpha<PI-TAU+limit)||
        (alpha>PI+TAU-limit&&alpha<PI+TAU+limit);
    endfunction
endpackage
