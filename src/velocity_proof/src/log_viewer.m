clear; close all; clc;
logFile = '~/Desktop/velocity_proof.txt';
logData = importdata(logFile, ' ');

m_p_raw = logData(logData(:,1)==1, 2:4).';
m_p_local = logData(logData(:,1)==2, 2:4).';
m_v = logData(logData(:,1)==3, 2:4).';
m_v_raw = logData(logData(:,1)==4, 2:4).';
m_v_local = logData(logData(:,1)==5, 2:4).';

l_p_raw = logData(logData(:,1)==6, 2:4).';
l_v = logData(logData(:,1)==7, 2:4).';
l_v_raw = logData(logData(:,1)==8, 2:4).';

figure(1);
for i=1:3
   subplot(1,3,i); plot(1:size(m_p_raw,2),m_p_raw(i,:),'-r'); hold on; 
   subplot(1,3,i); plot(1:size(m_p_local,2),m_p_local(i,:),'-b'); hold on; 
end

figure(2);
for i=1:3
   subplot(1,3,i); plot(1:size(m_p_raw,2),m_v(i,:),'-k'); hold on; 
   subplot(1,3,i); plot(1:size(m_p_raw,2),m_v_raw(i,:),'-r'); hold on; 
   %subplot(1,3,i); plot(1:size(m_p_local,2),m_v_local(i,:),'-b'); hold on; 
end

figure(3);
for i=1:3
   subplot(1,3,i); plot(1:size(m_p_raw,2),l_p_raw(i,:),'-k'); hold on; 
end

figure(4);
for i=1:3
   subplot(1,3,i); plot(1:size(m_p_raw,2),l_v(i,:),'-k'); hold on; 
   subplot(1,3,i); plot(1:size(m_p_raw,2),l_v_raw(i,:),'-r'); hold on; 
end