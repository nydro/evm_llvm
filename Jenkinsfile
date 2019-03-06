pipeline {
  agent none
  stages {
    stage('Make a build') {
      parallel {
        stage('linux') {
          agent {
            label 'linux'
          }
          steps {
            sh '''sudo rm -rf evmbuild
pwd'''
            sh '''cat /proc/cpuinfo
whoami'''
            sh 'mkdir -p evmbuild && cd evmbuild && sudo cmake $WORKSPACE -DLLVM_EXPERIMENTAL_TARGETS_TO_BUILD=EVM && sudo make -j 8'
            sh 'sudo rm -rf evmbuild'
          }
        }
        stage('macos') {
          agent {
            label 'macos'
          }
          steps {
            sh 'rm -rf evmbuild'
            sh '/usr/local/bin/brew install cmake'
            sh 'mkdir evmbuild && cd evmbuild && /usr/local/bin/cmake $WORKSPACE -DLLVM_EXPERIMENTAL_TARGETS_TO_BUILD=EVM && /usr/local/bin/cmake --build .'
            sh 'rm -rf evmbuild'
          }
        }
      }
    }
  }
}