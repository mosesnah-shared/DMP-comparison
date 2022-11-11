function y = myVonMises( s, ci, hi )

y = exp( hi * ( cos( s - ci ) - 1 ) );
end

