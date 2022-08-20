% 人工势场，斥力场函数
function pot= potential(i, j, T)
d=70; % 相对安全距离
if (i==j) % 自己对自己没有作用力
    pot=[0,0,0];
else if (norm(T(i,:)-T(j,:), 2) > 4*d)  %距离过远的个体没有排斥力
        pot=[0,0,0];
    else
    pot=2*d^2*(T(i,:)-T(j,:))/((norm(T(i,:)-T(j,:), 2))^3); % norm取模长，斥力与距离成二次方反比
    if (norm(T(i,:)-T(j,:), 2)<1.4*d)  % 距离过近的个体增加随机z方向的力，方便以一定高度差紧急避障
    pot=pot+100000*norm(pot)*sign(rand(1)-0.5)*[0,0,8];
    end
    end
end

end
