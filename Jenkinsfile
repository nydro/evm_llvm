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
            sh 'rm -rf evmbuild'
            sh 'mkdir -p evmbuild && cd evmbuild && cmake $WORKSPACE -DLLVM_EXPERIMENTAL_TARGETS_TO_BUILD=EVM && cmake --build . -- -j 8'
            sh 'rm -rf evmbuild'
          }
        }
        stage('macos') {
          agent {
            label 'macos'
          }
          steps {
            sh 'rm -rf evmbuild'
            sh '/usr/local/bin/brew install cmake'
            sh 'mkdir evmbuild && cd evmbuild && /usr/local/bin/cmake ../ -DLLVM_EXPERIMENTAL_TARGETS_TO_BUILD=EVM && /usr/local/bin/cmake --build . -- -j 8'
            sh 'rm -rf evmbuild'
          }
        }
      }
    }  
  }
}
