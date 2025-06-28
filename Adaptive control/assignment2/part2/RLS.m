function [teta , P] = RLS(U , V ,Y, teta , P , Nv, lamda)
U = U(:)' ;
V = V(:)' ;
phi = [U , V]' ;
K = P*phi*(lamda+phi'*P*phi)^(-1) ;
P = ((eye(Nv) - K*phi')*P)/lamda ;
teta = teta + K*(Y - phi'*teta ) ;
end