
## Appendix A:
### Calculate $F_{\~x}$ and $F_w$
$$
g(\~{x}, w) \stackrel{\mathrm{def}}{=}   f(x, u, w) \Delta{t}
\tag{1}
$$

$$
G(\~{x},g) \stackrel{\mathrm{def}}{=} \hat{x} \\= (\hat{x} \boxplus \~{x} \boxplus g(\~{x}, w)) 
\boxminus 
(\hat{x} \boxplus g(0, 0)) \\
\tag{2}
$$

$$
F_{\~x}=\frac{\partial G}{\partial \~x} + \frac{\partial G}{\partial g} \frac{\partial g}{\partial \~x} \tag{3}
$$

$$
F_{w}=\frac{\partial G}{\partial g} \frac{\partial g}{\partial w} \tag{4}
$$

$$
G= \hat x \boxplus \~x \boxplus g \boxminus \hat x \boxplus g_0 \tag{5}
$$

### Calculate $\frac{\partial G}{\partial \~x}$

if $\~x \in \mathbb{R}^n $
$$
\frac{\partial G}{\partial \~x}= \frac{\partial (\hat x + \~x + g - \hat x + g_0)}{\partial \~x}
=I \tag{7}
$$

if $\~x \in SO3 $
$$
\frac{\partial G}{\partial \~x} =  
\frac{\log (G(\~x)^{-1}G(\~x\exp{\delta}))}{\delta} \\
=\frac{ \log(( \exp(g)^{-1}exp({\delta})) \exp(g))}{\delta} 
=\frac{ \log(exp({\exp(g)^{-1}\delta}))}{\delta} \\
=\frac{ \exp(g)^{-1}\delta}{\delta} 
= \exp(g)^{-1} \\
= \exp(\omega - b_\omega \Delta t)^{-1} \\
= \exp(-(\omega - b_\omega )\Delta t)
\tag{8}
$$


So $\frac{\partial G}{\partial \~x}$
$$
\frac{\partial G}{\partial \~x} = 
\begin{bmatrix}
 I    & & & & & &\\  
  & \exp(\widehat{-(\omega - b_\omega )}\Delta t)   & & & & &\\  
  &  & I  & & & &\\
  & & & I & & & &\\
  & & & & I & & &\\
  & & & & & I & &\\
  & & & & & & I &\\
  & & & & & & & I\\
\end{bmatrix} 
\tag{9}
$$

### Calculate $\frac{\partial G}{\partial g}$

if $g \in \mathbb{R}^n $
$$
\frac{\partial G}{\partial g}= \frac{\partial (\hat x + \~x + g - \hat x + g_0)}{\partial g}
=I \tag{10}
$$


if $g \in SO3 $
$$
\frac{\partial G}{\partial g} =  
\frac{\log (G(g)^{-1}G(g\exp{\delta}))}{\delta} \\
=\frac{ \log(exp({\widehat{\delta}}))}{\delta} \\
= I 
\tag{11}
$$

So $\frac{\partial G}{\partial g}$
$$
\frac{\partial G}{\partial g} = 
\begin{bmatrix}
 I    & & & & & &\\  
  & I   & & & & &\\  
  &  & I  & & & &\\
  & & & I & & & &\\
  & & & & I & & &\\
  & & & & & I & &\\
  & & & & & & I &\\
  & & & & & & & I\\
\end{bmatrix} 
\tag{12}
$$


### Calculate $\frac{\partial g}{\partial \~x}$
$$
g(\~x,w) = f(x,u,w) \Delta t = 
\begin{bmatrix}
 v \\ 
\omega - b_{\omega}-n_{\omega} \\
 0 \\
 0 \\
 R(a-b_a-n_a)+g \\
 n_{b\omega} \\
 n_{ba} \\
 0
 \end{bmatrix} \Delta t 
 \tag{13}
 $$

 $$
 f(\hat{x} \boxplus \~{x},u,w) \Delta t= 
 \begin{bmatrix}
 \hat{v} + \~{v}\\ 
 \omega - \hat{b_{\omega}}- \~{b_{\omega}}-n_{\omega} \\
 0 \\
 0 \\
 \hat{R}\ \exp{(\~{\theta})}(a- \hat{b_a}- \~{b_a} -n_a)+ \hat{g} + \~{g} \\
 n_{b\omega} \\
 n_{ba} \\
 0
 \end{bmatrix} \Delta t
 \tag{14}
$$

####col 1
$$
\frac{\partial g}{\partial \~p} = 0_{3 \times 24}
\tag{15}
$$
####col 2
$$
\frac{\partial g}{\partial \~R} = 
\begin{bmatrix}
 0 \\ 
 0 \\
 0 \\
 0 \\
 -\hat{R}[a-\hat{b_a}]_{\times} \\
 0 \\
 0 \\
 0
 \end{bmatrix} \Delta t
 \tag{16}
$$

####col 3
$$
\frac{\partial g}{\partial \~R_{il}} = 0_{3 \times 24}
 \tag{20}
$$
####col 4
$$
\frac{\partial g}{\partial \~t_{il}} = 0_{3 \times 24}
 \tag{17}
$$
####col 5
$$
\frac{\partial g}{\partial \~v} = 
\begin{bmatrix}
 I \\ 
  0 \\
  0 \\
  0 \\
  0 \\
  0 \\
  0 \\
  0
 \end{bmatrix} \Delta t
  \tag{18}
$$
####col 6

$$
\frac{\partial g}{\partial \~{b_{\omega}}} = 
\begin{bmatrix}
0 \\
 -I \\ 
  0 \\
  0 \\
  0 \\
  0 \\
  0 \\
  0
 \end{bmatrix} \Delta t
   \tag{19}
$$
####col 7

$$
\frac{\partial g}{\partial \~{b_{a}}} = 
\begin{bmatrix}
0 \\
 0 \\ 
  0 \\
  0 \\
  -\hat{R} \\
  0 \\
  0 \\
  0
 \end{bmatrix} \Delta t
\tag{20}
$$
####col 8
$$
\frac{\partial g}{\partial \~g} = 
\begin{bmatrix}
  0 \\
  0 \\ 
  0 \\
  0 \\
  I \\
  0 \\
  0 \\
  0
 \end{bmatrix} \Delta t
 \tag{21}
$$

$$
\frac{\partial g}{\partial \~x} = 
\begin{bmatrix}
  & 0& 0& 0& 0& I& 0& 0 & 0 \\
  & 0& 0& 0& 0& 0& -I& 0 & 0 \\ 
  & 0& 0& 0& 0& 0& 0& 0 & 0 \\
  & 0& 0& 0& 0& 0& 0& 0 & 0 \\
  & 0& -\hat{R}[a-\hat{b_a}]_{\times}& 0& 0& 0& 0& -\hat{R} & I \\
  & 0& 0& 0& 0& 0& 0& 0 & 0 \\
  & 0& 0& 0& 0& 0& 0& 0 & 0 \\
  & 0& 0& 0& 0& 0& 0& 0 & 0
 \end{bmatrix} \Delta t
 \tag{22}
$$

### Calculate $F_{\~x}$
$$
\frac{\partial F}{\partial \~x} = \frac{\partial G}{\partial \~x} + \frac{\partial G}{\partial g} \frac{\partial g}{\partial \~x}=
\begin{bmatrix}
  & I& 0& 0& 0& I\Delta t& 0& 0 & 0 \\
  & 0& \exp(-(\omega - \hat b_\omega )\Delta t)& 0& 0& 0& -I\Delta t& 0 & 0 \\ 
  & 0& 0& I& 0& 0& 0& 0 & 0 \\
  & 0& 0& 0& I& 0& 0& 0 & 0 \\
  & 0& -\hat{R}[a-\hat{b_a}]_{\times} \Delta t& 0& 0& I& 0& -\hat{R}\Delta t & I\Delta t \\
  & 0& 0& 0& 0& 0& I & 0 & 0 \\
  & 0& 0& 0& 0& 0& 0& I & 0 \\
  & 0& 0& 0& 0& 0& 0& 0 & I
 \end{bmatrix} 
 \tag{23}
$$

### Calculate $\frac{\partial g}{\partial w}$
$$
g(\~x,w) = f(x,u,w) \Delta t = 
\begin{bmatrix}
 v \\ 
\omega - b_{\omega}-n_{\omega} \\
 0 \\
 0 \\
 R(a-b_a-n_a)+g \\
 n_{b\omega} \\
 n_{ba} \\
 0
 \end{bmatrix} \Delta t 
 \tag{24}
 $$

####col 1
$$
\frac{\partial g}{\partial n_\omega} = 
\begin{bmatrix}
 0 \\ 
 -I \\
 0 \\
 0 \\
 0 \\
 0 \\
 0 \\
 0
 \end{bmatrix} \Delta t
 \tag{25}
$$
####col 2
$$
\frac{\partial g}{\partial n_a} = 
\begin{bmatrix}
 0 \\ 
 0 \\
 0 \\
 0 \\
 -\hat{R} \\
 0 \\
 0 \\
 0
 \end{bmatrix} \Delta t
 \tag{26}
$$

####col 3
$$
\frac{\partial g}{\partial n_{b\omega}} = 
\begin{bmatrix}
 0 \\ 
 0 \\
 0 \\
 0 \\
 0\\
 I \\
 0 \\
 0
 \end{bmatrix} \Delta t
 \tag{27}
$$
####col 4
$$
\frac{\partial g}{\partial n_{ba}} =
\begin{bmatrix}
 0 \\ 
 0 \\
 0 \\
 0 \\
 0 \\
 0 \\
 I \\
 0
 \end{bmatrix} \Delta t
 \tag{28}
$$

$$
\frac{\partial g}{\partial \~x} = 
\begin{bmatrix}
  & 0& 0& 0& 0 \\
  &-I& 0& 0& 0 \\ 
  & 0& 0& 0& 0 \\
  & 0& 0& 0& 0 \\
  & 0& -\hat{R}& 0& 0 \\
  & 0& 0& I& 0 \\
  & 0& 0& 0& I \\
  & 0& 0& 0& 0
 \end{bmatrix} \Delta t
  \tag{29}
$$

### Calculate $F_{w}$

$$
F_{w}= \frac{\partial G}{\partial g} \frac{\partial g}{\partial w} =
\begin{bmatrix}
  & 0& 0& 0& 0 \\
  &-I& 0& 0& 0 \\ 
  & 0& 0& 0& 0 \\
  & 0& 0& 0& 0 \\
  & 0& -\hat{R}& 0& 0 \\
  & 0& 0& I& 0 \\
  & 0& 0& 0& I \\
  & 0& 0& 0& 0
 \end{bmatrix} \Delta t
 \tag{30}
$$

## Appendix B:

$$
\argmin_{\substack{\~{x}}} ( \|z + H\~{x}\|_{R^{-1}}^2 + \|d + J\~{x}\|_{\hat{P}^{-1}}^2  )
\tag{1}
$$

$$
\argmin_{\substack{\~{x}}} ( \frac{1}{2} \mathcal{R}^T \Sigma \mathcal{R} )
\tag{2}
$$

$$
\mathcal{R}  = 
\begin{bmatrix}
 z + H\~{x}       \\  
 d + J\~{x}       \\  
\end{bmatrix}
\tag{3}
$$

$$
\Sigma  = 
\begin{bmatrix}
 R^{-1} &   \\  
 & \hat{P}^{-1}  \\  
\end{bmatrix} 
\tag{4}
$$

$$
\mathcal{J} =
\begin{bmatrix}
 H \\  
 J \\  
\end{bmatrix} 
\tag{5}
$$

$$
\mathcal{H} = \mathcal{J}^T \Sigma \mathcal{J} 
\tag{6}
$$

$$
\mathcal{g} = \mathcal{J}^T \Sigma \mathcal{R} 
\tag{7}
$$

$$
\mathcal{H} = H^T R^{-1} H + J^T \hat{P}^{-1} J
= H^T R^{-1} H + P^{-1}
\tag{8}
$$

$$
\mathcal{H}^{-1} = (H^T R^{-1} H + P^{-1})^{-1} \\
=P - PH^T(R+HPH^T)^{-1}HP \\
=P - KHP \\
=(I - KH)P \\
=KRH^{-T}
\tag{9}
$$

https://en.wikipedia.org/wiki/Woodbury_matrix_identity
$$
\Delta{\~{x}} = - \mathcal{H}^{-1} \mathcal{g} \\
=-\mathcal{H}^{-1} \mathcal{J}^T \Sigma \mathcal{R} \\
= -\mathcal{H}^{-1}(H^TR^{-1}(z+H\~{x})) -\mathcal{H}^{-1}(J^T\hat{P}^{-1}(d+J\~{x})) \\
= -KRH^{-T}(H^TR^{-1}(z+H\~{x})) -(I - KH)(J^{-1} \hat{P} J^{-T})(J^T\hat{P}^{-1}(d+J\~{x})) \\
= -K(z+H\~{x}) - (I - KH)(J^{-1} (d+J\~{x})) \\
= -Kz-KH\~{x} - (I - KH)J^{-1} d - \~{x} + KH\~{x} \\
= -Kz - (I - KH)J^{-1} d - \~{x} \\
$$

$$
\~{x}^{ \kappa + 1} = \~{x}^{ \kappa } \boxplus \Delta{\~{x}} \\
\~{x}^{ \kappa + 1} = -Kz - (I - KH)J^{-1} d
$$

$$
\bar{P}=\mathcal{H}^{-1}=(I - KH)P
$$
https://onlinelibrary.wiley.com/doi/pdf/10.1002/9780470824566.app1




