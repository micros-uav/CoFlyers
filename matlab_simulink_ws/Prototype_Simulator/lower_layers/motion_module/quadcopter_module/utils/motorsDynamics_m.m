function dw = motorsDynamics_m(t,w,w_d,T_re)

dw = T_re*(w_d - w);

end