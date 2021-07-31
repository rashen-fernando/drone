sampling_time = 35/1000;  %sampling time for now on esp 32 board.this depends on the code size
                       %when the final code is done run this code with new
                       %sampling time and generate coefficients and
                       %implement them to the esp32 board.
Frequency = 1/sampling_time;
pass_band_freq = 70;
stop_band_freq = 100;

%passband_attenuation
Ap = 0.89;
%stopband_attenuation
As = 0.17;

%omega_pass_freq 
wp = pass_band_freq * 2 * pi/Frequency;
%omega_stop_freq 
ws = stop_band_freq * 2 * pi/Frequency;

%error=10;
%N=0;
% for n=1:10
%     omega_cutoff_from_pass=omega_pass_freq/(1/(passband_attenuation^2) -1 )^(1/(2*n));
%     omega_cutoff_from_stop=omega_stop_freq/(1/(stopband_attenuation^2) -1 )^(1/(2*n));
%     
%     if abs(omega_cutoff_from_pass-omega_cutoff_from_stop)<error
%         error=abs(omega_cutoff_from_pass-omega_cutoff_from_stop);
%         N=n;
%         wc=omega_cutoff_from_pass;
%     else
%         error=error;
%     end
%     
% end

N = log((1/As^2 -1)/(1/Ap^2 -1))/(2*log(ws/wp));
wc = wp/(1/Ap^2 -1)^(1/(2*N));
N = round(N);

k=0:N-1;
sk = wc*exp((complex(0,1)*pi)*(2*k+N+1)/(2*N));


%tf_denominator
tf_den = 1;
for i=1:N
    tf_den = conv(tf_den,[1 -sk(i)]);
end

%trrasfer function denominator coefficients
tf_den_coeffs = real(tf_den);

sys=tf(1,tf_den_coeffs);

n=0:10;
h=0;
for k = 1:N
    h = h + exp( n*sk(k)* sampling_time);
end

coeffs_of_hn=real(h)./sum(real(h))




                       