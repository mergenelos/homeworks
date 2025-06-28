function [R , S , Ac_result] = Diophantine(A , B , Ac)

n = numel(A)-1 ; % degree of A polynomial
m = numel(B)-1 ; % degree of B polynomial
B = [zeros(1,n-m) , B] ;

E = zeros(2*n , 2*n) ;

for i = 1:n
   E(: , i) = [zeros(i-1 ,1) ; A' ; zeros(n-i , 1) ] ;  
   E(: , i+n) = [zeros(i-1 ,1) ; B' ; zeros(n-i , 1) ] ;  
end

nc = numel(Ac)-1 ;
Ac = [zeros(1,2*n-(nc+1)) , Ac] ;

RS = E\ Ac' ;

R = RS(1:n); 
S = RS(n+1:end); 

% Ac_result = A*R+B*S ;
AR = conv(A,R) ;
BS = conv(B,S) ;
Ac_result = poly2sym(AR) + poly2sym(BS) ;
Ac_result = sym2poly(Ac_result);
         
S = S(:)' ;
R = R(:)' ;
    
end