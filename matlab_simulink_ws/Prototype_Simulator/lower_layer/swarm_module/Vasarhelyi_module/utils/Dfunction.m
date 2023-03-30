function D = Dfunction(r,a,p)
D = r*0;
temp = r<a/p/p;
condition1 = r>0 & temp;
condition2 = ~(temp);
D(condition1) = r(condition1)*p;
D(condition2) = sqrt(2*a*r(condition2) - a*a/(p*p));
end