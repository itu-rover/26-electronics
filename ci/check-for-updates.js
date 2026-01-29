const gitConfig = require("git-config");

module.exports = async function (github, context, core) {
    const modules = gitConfig.sync(".gitmodules");

    let releases = [];
    for (const [_, module] of Object.entries(modules)) {
        if (!module.url.startsWith("https://github.com/"))
            continue;
        const parsedUrl = URL.parse(module.url);
        const pathParts = parsedUrl.pathname.split('/').filter(part => part.length > 0);

        if (pathParts.length != 2) {
            core.error(`Repo URL ${module.url} invalid`);
            continue;
        }

        const owner = pathParts[0];
        const repo = pathParts[1].replace(/\.git$/, "");

        try {
            const { data: { tag_name: tagName } } = await github.rest.repos.getLatestRelease({ owner, repo });

            const refName = `tags/${tagName}`;
            const { data: ref } = await github.rest.git.getRef({
                owner,
                repo,
                ref: refName,
            });
            const { sha } = ref.object;
            core.info(`submodule ${owner}/${repo} latest version: ${tagName} (${sha})`);
            releases.push({ sha, tagName, path: module.path });
        } catch (e) {
            if (e.status !== 404)
                throw e;
            core.warning(`submodule ${owner}/${repo} doesn't have releases`);
            continue;
        }
    }

    core.setOutput('submodules', JSON.stringify({ submodule: releases }));
}
