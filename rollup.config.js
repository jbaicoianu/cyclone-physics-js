import { nodeResolve } from '@rollup/plugin-node-resolve';

export default {
  input: 'cyclone.js',
  output: {
    dir: 'build',
    format: 'es'
  },
  plugins: [nodeResolve()],
};
