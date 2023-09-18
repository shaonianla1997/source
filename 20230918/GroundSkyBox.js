import {
  BoxGeometry,
  Cartesian3,
  defaultValue,
  defined,
  destroyObject,
  GeometryPipeline,
  Matrix3,
  Matrix4,
  Transforms,
  VertexFormat,
  BufferUsage,
  DrawCommand,
  RenderState,
  VertexArray,
  BlendingState,
  SceneMode,
  ShaderProgram,
  ShaderSource,
} from 'cesium';
export class GroundSkyBox {
  constructor({ viewer, sources, show = true }) {
    this.sources = sources;
    this.viewer = viewer;
    this.show = show;
    this.viewer.scene.skyAtmosphere.show = false;
    this.rewriteSkyBox();
  }
  rewriteSkyBox() {
    const SkyBoxFS = `
    	
#define SHADERTOY


#if defined(GL_ES) || defined(GL_SHADING_LANGUAGE_VERSION)
#define _in(T) const in T
#define _inout(T) inout T
#define _out(T) out T
#define _begin(type) type (
#define _end )
#define _mutable(T) T
#define _constant(T) const T
#define mul(a, b) (a) * (b)
precision mediump float;
#endif

#if defined(__cplusplus) || defined(SHADERTOY)
#define u_res czm_viewport.zw
#define u_time czm_frameNumber / 200.0
#define u_mouse czm_viewport.zw / 2
#endif

uniform sampler2D iChannel0; 


#define PI 3.14159265359

struct ray_t {
	vec3 origin;
	vec3 direction;
};
#define BIAS 1e-4 // small offset to avoid self-intersections

struct sphere_t {
	vec3 origin;
	float radius;
	int material;
};

struct plane_t {
	vec3 direction;
	float distance;
	int material;
};

struct hit_t {
	float t;
	int material_id;
	vec3 normal;
	vec3 origin;
};
#define max_dist 1e8
_constant(hit_t) no_hit = _begin(hit_t)
	float(max_dist + 1e1), // 'infinite' distance
	-1, // material id
	vec3(0., 0., 0.), // normal
	vec3(0., 0., 0.) // origin
_end;

ray_t get_primary_ray(
	_in(vec3) cam_local_point,
	_inout(vec3) cam_origin,
	_inout(vec3) cam_look_at
){
	vec3 fwd = normalize(cam_look_at - cam_origin);
	vec3 up = vec3(0, 1, 0);
	vec3 right = cross(up, fwd);
	up = cross(fwd, right);

	ray_t r = _begin(ray_t)
		cam_origin,
		normalize(fwd + up * cam_local_point.y + right * cam_local_point.x)
	_end;
	return r;
}

_constant(mat3) mat3_ident = mat3(1, 0, 0, 0, 1, 0, 0, 0, 1);


mat2 rotate_2d(
	_in(float) angle_degrees
){
	float angle = radians(angle_degrees);
	float _sin = sin(angle);
	float _cos = cos(angle);
	return mat2(_cos, -_sin, _sin, _cos);
}

mat3 rotate_around_z(
	_in(float) angle_degrees
){
	float angle = radians(angle_degrees);
	float _sin = sin(angle);
	float _cos = cos(angle);
	return mat3(_cos, -_sin, 0, _sin, _cos, 0, 0, 0, 1);
}

mat3 rotate_around_y(
	_in(float) angle_degrees
){
	float angle = radians(angle_degrees);
	float _sin = sin(angle);
	float _cos = cos(angle);
	return mat3(_cos, 0, _sin, 0, 1, 0, -_sin, 0, _cos);
}

mat3 rotate_around_x(
	_in(float) angle_degrees
){
	float angle = radians(angle_degrees);
	float _sin = sin(angle);
	float _cos = cos(angle);
	return mat3(1, 0, 0, 0, _cos, -_sin, 0, _sin, _cos);
}

vec3 linear_to_srgb(
	_in(vec3) color
){
	const float p = 1. / 2.2;
	return vec3(pow(color.r, p), pow(color.g, p), pow(color.b, p));
}
vec3 srgb_to_linear(
	_in(vec3) color
){
	const float p = 2.2;
	return vec3(pow(color.r, p), pow(color.g, p), pow(color.b, p));
}


float checkboard_pattern(
	_in(vec2) pos,
	_in(float) scale
){
	vec2 pattern = floor(pos * scale);
	return mod(pattern.x + pattern.y, 2.0);
}

float band (
	_in(float) start,
	_in(float) peak,
	_in(float) end,
	_in(float) t
){
	return
	smoothstep (start, peak, t) *
	(1. - smoothstep (peak, end, t));
}
void fast_orthonormal_basis(
	_in(vec3) n,
	_out(vec3) f,
	_out(vec3) r
){
	float a = 1. / (1. + n.z);
	float b = -n.x*n.y*a;
	f = vec3(1. - n.x*n.x*a, b, -n.x);
	r = vec3(b, 1. - n.y*n.y*a, -n.y);
}

void intersect_sphere(
	_in(ray_t) ray,
	_in(sphere_t) sphere,
	_inout(hit_t) hit
){
	vec3 rc = sphere.origin - ray.origin;
	float radius2 = sphere.radius * sphere.radius;
	float tca = dot(rc, ray.direction);
	if (tca < 0.) return;

	float d2 = dot(rc, rc) - tca * tca;
	if (d2 > radius2) return;

	float thc = sqrt(radius2 - d2);
	float t0 = tca - thc;
	float t1 = tca + thc;

	if (t0 < 0.) t0 = t1;
	if (t0 > hit.t) return;

	vec3 impact = ray.origin + ray.direction * t0;

	hit.t = t0;
	hit.material_id = sphere.material;
	hit.origin = impact;
	hit.normal = (impact - sphere.origin) / sphere.radius;
}
void intersect_plane(
	_in(ray_t) ray,
	_in(plane_t) p,
	_inout(hit_t) hit
){
	float denom = dot(p.direction, ray.direction);
	if (denom < 1e-6) return;

	vec3 P0 = vec3(p.distance, p.distance, p.distance);
	float t = dot(P0 - ray.origin, p.direction) / denom;
	if (t < 0. || t > hit.t) return;
	
	hit.t = t;
	hit.material_id = p.material;
	hit.origin = ray.origin + ray.direction * t;
	hit.normal = faceforward(p.direction, ray.direction, p.direction);
}

// ----------------------------------------------------------------------------
// Volumetric utilities
// ----------------------------------------------------------------------------

float isotropic_phase_func(float mu)
{
	return 1. / 4. * PI;
}

float rayleigh_phase_func(float mu)
{
	return 3. * (1. + mu*mu) / (16. * PI);
}

float henyey_greenstein_phase_func(float mu)
{
	const float g = 0.76;

	return (1. - g*g) / ((4. + PI) * pow(1. + g*g - 2.*g*mu, 1.5));
}

float schlick_phase_func(float mu)
{
	const float g = 0.76;
	const float k = 1.55*g - 0.55 * (g*g*g);

	return (1. - k*k) / (4. * PI * (1. + k*mu) * (1. + k*mu));
}

struct volume_sampler_t {
	vec3 origin; // start of ray
	vec3 pos; // current pos of acccumulation ray
	float height;

	float coeff_absorb;
	float T; // transmitance

	vec3 C; // color
	float alpha;
};

volume_sampler_t begin_volume(
	_in(vec3) origin,
	_in(float) coeff_absorb
){
	volume_sampler_t v = _begin(volume_sampler_t)
		origin, origin, 0.,
		coeff_absorb, 1.,
		vec3(0., 0., 0.), 0.
	_end;
	return v;
}

float illuminate_volume(
	_inout(volume_sampler_t) vol,
	_in(vec3) V,
	_in(vec3) L
);

void integrate_volume(
	_inout(volume_sampler_t) vol,
	_in(vec3) V,
	_in(vec3) L,
	_in(float) density,
	_in(float) dt
){
	// change in transmittance (follows Beer-Lambert law)
	float T_i = exp(-vol.coeff_absorb * density * dt);
	// Update accumulated transmittance
	vol.T *= T_i;
	// integrate output radiance (here essentially color)
	vol.C += vol.T * illuminate_volume(vol, V, L) * density * dt;
	// accumulate opacity
	vol.alpha += (1. - T_i) * (1. - vol.alpha);
}


#define cld_march_steps (50)
#define cld_coverage (.3125)
#define cld_thick (90.)
#define cld_absorb_coeff (1.)
#define cld_wind_dir vec3(0, 0, -u_time * .2)
#define cld_sun_dir normalize(vec3(0, 0/*abs(sin(u_time * .3))*/, -1))
_mutable(float) coverage_map;


float hash(
	_in(float) n
){
	return fract(sin(n)*753.5453123);
}

float noise_iq(
	_in(vec3) x
){
	vec3 p = floor(x);
	vec3 f = fract(x);
	f = f*f*(3.0 - 2.0*f);

    vec2 uv = (p.xy + vec2(37.0, 17.0)*p.z) + f.xy;
	vec2 rg = textureLod( iChannel0, (uv+.5)/256., 0.).yx;
	return mix(rg.x, rg.y, f.z);
}

#define gnoise(x) noise_iq(x)

vec3 mod289(vec3 x) {
  return x - floor(x * (1.0 / 289.0)) * 289.0;
}

vec4 mod289(vec4 x) {
  return x - floor(x * (1.0 / 289.0)) * 289.0;
}

vec4 permute(vec4 x) {
     return mod289(((x*34.0)+1.0)*x);
}

vec4 taylorInvSqrt(vec4 r)
{
  return 1.79284291400159 - 0.85373472095314 * r;
}

float snoise(vec3 v)
  { 
  const vec2  C = vec2(1.0/6.0, 1.0/3.0) ;
  const vec4  D = vec4(0.0, 0.5, 1.0, 2.0);

// First corner
  vec3 i  = floor(v + dot(v, C.yyy) );
  vec3 x0 =   v - i + dot(i, C.xxx) ;

// Other corners
  vec3 g = step(x0.yzx, x0.xyz);
  vec3 l = 1.0 - g;
  vec3 i1 = min( g.xyz, l.zxy );
  vec3 i2 = max( g.xyz, l.zxy );

  vec3 x1 = x0 - i1 + C.xxx;
  vec3 x2 = x0 - i2 + C.yyy; // 2.0*C.x = 1/3 = C.y
  vec3 x3 = x0 - D.yyy;      // -1.0+3.0*C.x = -0.5 = -D.y

// Permutations
  i = mod289(i); 
  vec4 p = permute( permute( permute( 
             i.z + vec4(0.0, i1.z, i2.z, 1.0 ))
           + i.y + vec4(0.0, i1.y, i2.y, 1.0 )) 
           + i.x + vec4(0.0, i1.x, i2.x, 1.0 ));

  float n_ = 0.142857142857; // 1.0/7.0
  vec3  ns = n_ * D.wyz - D.xzx;

  vec4 j = p - 49.0 * floor(p * ns.z * ns.z);  //  mod(p,7*7)

  vec4 x_ = floor(j * ns.z);
  vec4 y_ = floor(j - 7.0 * x_ );    // mod(j,N)

  vec4 x = x_ *ns.x + ns.yyyy;
  vec4 y = y_ *ns.x + ns.yyyy;
  vec4 h = 1.0 - abs(x) - abs(y);

  vec4 b0 = vec4( x.xy, y.xy );
  vec4 b1 = vec4( x.zw, y.zw );

  vec4 s0 = floor(b0)*2.0 + 1.0;
  vec4 s1 = floor(b1)*2.0 + 1.0;
  vec4 sh = -step(h, vec4(0, 0, 0, 0));

  vec4 a0 = b0.xzyw + s0.xzyw*sh.xxyy ;
  vec4 a1 = b1.xzyw + s1.xzyw*sh.zzww ;

  vec3 p0 = vec3(a0.xy,h.x);
  vec3 p1 = vec3(a0.zw,h.y);
  vec3 p2 = vec3(a1.xy,h.z);
  vec3 p3 = vec3(a1.zw,h.w);

  vec4 norm = taylorInvSqrt(vec4(dot(p0,p0), dot(p1,p1), dot(p2, p2), dot(p3,p3)));
  p0 *= norm.x;
  p1 *= norm.y;
  p2 *= norm.z;
  p3 *= norm.w;

  vec4 m = max(0.6 - vec4(dot(x0,x0), dot(x1,x1), dot(x2,x2), dot(x3,x3)), 0.0);
  m = m * m;
  return 42.0 * dot( m*m, vec4( dot(p0,x0), dot(p1,x1), 
                                dot(p2,x2), dot(p3,x3) ) );
  }
#define noise(x) snoise(x)

#define DECL_FBM_FUNC(_name, _octaves, _basis) float _name(_in(vec3) pos, _in(float) lacunarity, _in(float) init_gain, _in(float) gain) { vec3 p = pos; float H = init_gain; float t = 0.; for (int i = 0; i < _octaves; i++) { t += _basis * H; p *= lacunarity; H *= gain; } return t; }

DECL_FBM_FUNC(fbm, 4, noise(p))
DECL_FBM_FUNC(fbm_clouds, 5, abs(noise(p)))

vec3 render_sky_color(
	_in(vec3) eye_dir
){
	_constant(vec3) sun_color = vec3(1., .7, .55);
	float sun_amount = max(dot(eye_dir, cld_sun_dir), 0.);

	vec3 sky = mix(vec3(.0, .1, .4), vec3(.3, .6, .8), 1.0 - eye_dir.y);
	sky += sun_color * min(pow(sun_amount, 1500.0) * 5.0, 1.0);
	sky += sun_color * min(pow(sun_amount, 10.0) * .6, 1.0);

	return sky;
}

float density_func(
	_in(vec3) pos,
	_in(float) h
){
	vec3 p = pos * .001 + cld_wind_dir;
	float dens = fbm_clouds(p * 2.032, 2.6434, .5, .5);
	
	dens *= smoothstep (cld_coverage, cld_coverage + .035, dens);

	return dens;
}

float illuminate_volume(
	_inout(volume_sampler_t) cloud,
	_in(vec3) V,
	_in(vec3) L
){
	return exp(cloud.height) / 1.95;
}

vec4 render_clouds(
	_in(ray_t) eye
){
	const int steps = cld_march_steps;
	const float march_step = cld_thick / float(steps);

	vec3 projection = eye.direction / eye.direction.y;
	vec3 iter = projection * march_step;

	float cutoff = dot(eye.direction, vec3(0, 1, 0));

	volume_sampler_t cloud = begin_volume(
		eye.origin + projection * 100.,
		cld_absorb_coeff);


	for (int i = 0; i < steps; i++) {
		cloud.height = (cloud.pos.y - cloud.origin.y)
			/ cld_thick;
		float dens = density_func(cloud.pos, cloud.height);

		integrate_volume(
			cloud,
			eye.direction, cld_sun_dir,
			dens, march_step);

		cloud.pos += iter;

		if (cloud.alpha > .999) break;
	}

	return vec4(cloud.C, cloud.alpha * smoothstep(.0, .2, cutoff));
}

void setup_camera(
	_inout(vec3) eye,
	_inout(vec3) look_at
){
	eye = vec3(0, 1., 0);
	look_at = vec3(0, 1.6, -1);
}

void setup_scene()
{
}

vec3 render(
	_in(ray_t) eye_ray,
	_in(vec3) point_cam
){
	vec3 sky = render_sky_color(eye_ray.direction);
	if (dot(eye_ray.direction, vec3(0, 1, 0)) < 0.05) return sky;

	vec4 cld = render_clouds(eye_ray);
	vec3 col = mix(sky, cld.rgb, cld.a);

	return col;
}

#define FOV 3.1 // 45 degrees

void main(){
	vec2 aspect_ratio = vec2(u_res.x / u_res.y, 1);

	vec3 color = vec3(0, 0, 0);

	vec3 eye, look_at;
	setup_camera(eye, look_at);

	setup_scene();

	vec2 point_ndc = gl_FragCoord.xy / u_res.xy;

	vec3 point_cam = vec3(
		(2.0 * point_ndc - 1.0) * aspect_ratio * FOV,
		-1.0);

	ray_t ray = get_primary_ray(point_cam, eye, look_at);

	color += render(ray, point_cam);

	gl_FragColor = vec4(linear_to_srgb(color), 1);
}



      `;
    //顶点着色器有修改，主要是乘了一个旋转矩阵
    const SkyBoxVS =
      'attribute vec3 position;\n\
        varying vec3 v_texCoord;\n\
        uniform mat3 u_rotateMatrix;\n\
        void main()\n\
        {\n\
        vec3 p = czm_viewRotation * u_rotateMatrix * (czm_temeToPseudoFixed * (czm_entireFrustum.y * position));\n\
        gl_Position = czm_projection * vec4(p, 1.0);\n\
        v_texCoord = position.xyz;\n\
        }\n\
        ';
    /**
     * 为了兼容高版本的Cesium，因为新版cesium中getRotation被移除
     */
    if (!defined(Matrix4.getRotation)) {
      Matrix4.getRotation = Matrix4.getMatrix3;
    }
    function SkyBoxOnGround(options) {
      /**
       * 近景天空盒
       * @type Object
       * @default undefined
       */
      this.sources = options.sources;
      this._sources = undefined;

      /**
       * Determines if the sky box will be shown.
       *
       * @type {Boolean}
       * @default true
       */
      this.show = defaultValue(options.show, true);

      this._command = new DrawCommand({
        modelMatrix: Matrix4.clone(Matrix4.IDENTITY),
        owner: this,
      });

      this._attributeLocations = undefined;
      this._useHdr = undefined;
    }
    const skyboxMatrix3 = new Matrix3();
    SkyBoxOnGround.prototype.update = function (frameState, useHdr) {
      if (!this.show) {
        return undefined;
      }

      if (frameState.mode !== SceneMode.SCENE3D && frameState.mode !== SceneMode.MORPHING) {
        return undefined;
      }

      if (!frameState.passes.render) {
        return undefined;
      }

      const context = frameState.context;

      const command = this._command;
      const _iChannel0 = require('@/assets/iChannel2.png');
      command.modelMatrix = Transforms.eastNorthUpToFixedFrame(frameState.camera._positionWC);
      if (!defined(command.vertexArray)) {
        command.uniformMap = {
          u_rotateMatrix: function () {
            return Matrix4.getRotation(command.modelMatrix, skyboxMatrix3);
          },
          iChannel0: function () {
            return _iChannel0;
          },
        };

        const geometry = BoxGeometry.createGeometry(
          BoxGeometry.fromDimensions({
            dimensions: new Cartesian3(2.0, 2.0, 2.0),
            vertexFormat: VertexFormat.POSITION_ONLY,
          }),
        );
        const attributeLocations = (this._attributeLocations = GeometryPipeline.createAttributeLocations(geometry));

        command.vertexArray = VertexArray.fromGeometry({
          context: context,
          geometry: geometry,
          attributeLocations: attributeLocations,
          bufferUsage: BufferUsage._DRAW,
        });

        command.renderState = RenderState.fromCache({
          blending: BlendingState.ALPHA_BLEND,
        });
      }

      if (!defined(command.shaderProgram) || this._useHdr !== useHdr) {
        const fs = new ShaderSource({
          defines: [useHdr ? 'HDR' : ''],
          sources: [SkyBoxFS],
        });
        command.shaderProgram = ShaderProgram.fromCache({
          context: context,
          vertexShaderSource: SkyBoxVS,
          fragmentShaderSource: fs,
          attributeLocations: this._attributeLocations,
        });
        this._useHdr = useHdr;
      }

      return command;
    };
    SkyBoxOnGround.prototype.isDestroyed = function () {
      return false;
    };
    SkyBoxOnGround.prototype.destroy = function () {
      const command = this._command;
      command.vertexArray = command.vertexArray && command.vertexArray.destroy();
      command.shaderProgram = command.shaderProgram && command.shaderProgram.destroy();
      return destroyObject(this);
    };
    this.viewer.scene.skyBox = new SkyBoxOnGround(this);
  }
}
